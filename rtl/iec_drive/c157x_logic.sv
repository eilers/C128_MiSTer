//-------------------------------------------------------------------------------
//
// Reworked and adapted to MiSTer by Sorgelig@MiSTer (07.09.2018)
//
// Extended with 157x models by Erik Scheffers
//
//-------------------------------------------------------------------------------
//
// Model 1541B / 1570 / 1571 
//
module c157x_logic #(DRIVE)
(
	input        clk,
	input        reset,
	input  [1:0] drv_mode,      // 00: 1541, 01: 1570, 10: 1571, (11: 1571CR todo)

	input        wd_ce,
	input  [1:0] ph2_r,
	input  [1:0] ph2_f,

	output       act,		    // activity LED

	// serial bus
	input        iec_clk_in,
	input        iec_data_in,
	input        iec_atn_in,
	input        iec_fclk_in,
	output       iec_clk_out,
	output       iec_data_out,
	output       iec_fclk_out,

	input        ext_en,
	output[14:0] rom_addr,
	input  [7:0] rom_data,

	// parallel bus
	input  [7:0] par_data_in,
	input        par_stb_in,
	output [7:0] par_data_out,
	output       par_stb_out,

	// drive control signals
	output       hinit,         // init head buffer
	input        hclk,          // bit clock
	input        hf,            // signal from head
	output       ht,            // signal to head

	output       side,          // disk side  
	input        wps_n,		    // write-protect sense
	
	output       mode,          // GCR mode (0=write, 1=read)
	output       wgate,         // MFM wgate (0=read, 1=write)
	output       fdc_busy,      // WD1770 busy
	output [1:0] stp,			 	 // stepper motor control
	output       mtr,			 	 // drive motor on/off
	output [1:0] freq,		    // motor frequency
	input        tr00_sense,    // track 0 sense
	input        index_sense,   // index pulse
	input        drive_enable,  // sd busy
	input        disk_present,

	input        img_mfm,       // mfm supported by disk image

	// Linear D64/D71 sector-image GCR path. This bypasses the raw-track 64H156
	// input while preserving the original VIA and DOS behavior.
	input        sector_gcr_enable,
	input  [7:0] sector_gcr_dout,
	input        sector_gcr_sync_n,
	input        sector_gcr_byte_n,
	output [7:0] sector_gcr_din,

	// MEGA65 read-only diagnostics, see the dos_diag block at the end of this file.
	output [1919:0] dos_diag
);

// clock control
// MEGA65 port: 0 on the FPGA, X in xsim without this, and an X in accl blocks ena_f.
reg [2:0] accl = 0;
reg [7:0] zp_state = 0;
always @(posedge clk)
begin
	if (~|drv_mode)
		accl <= 3'b000;
	else if (ph2_r[accl[1]]) 
		accl <= {accl[1:0],accl_ctl};
end

wire halt  = accl[0]^accl[2];
wire ena_f = ph2_f[accl[1]] & ~halt;
wire ena_r = ph2_r[accl[1]] & ~halt;

// cpu signal decode
assign rom_addr = cpu_a[14:0];

//same decoder as on real HW
wire [3:0] ls42 = {cpu_a[15], cpu_a[12:10]};   
wire ram_cs    = |drv_mode ? cpu_a[15:12] == 0 || cpu_a[15:13] == 3            : ls42 == 0 || ls42 == 1;
wire via1_cs   = |drv_mode ? cpu_a[15:12] == 1 && cpu_a[10] == 0               : ls42 == 6;
wire via2_cs   = |drv_mode ? cpu_a[15:12] == 1 && cpu_a[10] == 1               : ls42 == 7;
wire wd_cs     = |drv_mode ? cpu_a[15:13] == 1                                 : 1'b0;
wire cia_cs    = |drv_mode ? cpu_a[15:13] == 2 && (~&drv_mode | cpu_a[4] == 0) : 1'b0;
// wire scrram_cs = &drv_mode ? cpu_a[15:13] == 2 && cpu_a[4] == 1                : 1'b0;
wire rom_cs    = cpu_a[15];

wire  [7:0] cpu_di =
	!cpu_rw    ? cpu_do :
	 ram_cs    ? ram_do :
	 via1_cs   ? via1_cpu_do :
	 via2_cs   ? via2_do :
	 wd_cs     ? wd_do :
	 cia_cs    ? cia_do :
	//  scrram_cs ? scrram_do :
	 extram_cs ? extram_do :
	 rom_cs    ? rom_data :
	 8'hFF;

wire [23:0] cpu_a;
wire  [7:0] cpu_do;
wire        cpu_rw;
wire        cpu_irq_n = ~(via1_irq | via2_irq) & cia_irq_n;

// MEGA65 port: the 64H156 signals below are declared here rather than next to their
// instance further down, because the port connections in this file name them first
// and Vivado answers that by creating 1-bit implicit nets. gcr_do is the byte the
// read head delivers to VIA2 port A, so truncating it to one bit meant the 1541/1571
// could never read anything off a disk.
wire [7:0] gcr_do;
wire       sync_n, byte_n, dgcr_we;
wire       gcr_ht, gcr_hinit;
wire [7:0] h156_do;
wire       h156_sync_n, h156_byte_n;

T65 cpu
(
	.mode(2'b00),
	.res_n(~reset),
	.enable(ena_f),
	.clk(clk),
	.rdy(1'b1),
	.abort_n(1'b1),
	.irq_n(cpu_irq_n),
	.nmi_n(1'b1),
	.so_n(byte_n),
	.r_w_n(cpu_rw),
	.A(cpu_a),
	.DI(cpu_di),
	.DO(cpu_do)
);

// optional 8k RAM at $8000-$9FFF for custom roms

wire extram_cs = ext_en && (cpu_a[15:13] == 'b100);

wire [7:0] extram_do;
iecdrv_mem #(.DATAWIDTH(8), .ADDRWIDTH(13), .WRITE_B(0)) extram
(
	.clock_a(clk),
	.address_a(cpu_a[12:0]),
	.data_a(cpu_do),
	.wren_a(ena_r & ~cpu_rw & extram_cs),

	// MEGA65 port: port B only ever reads. Leaving wren_b/data_b unconnected makes Vivado
	// infer a true dual-port RAM whose two write ports share one address, which it reports
	// as [Synth 8-5796] and whose collision behaviour is undefined. Tying them off keeps
	// the inferred RAM a simple dual-port and matches the simulation exactly.
	.clock_b(clk),
	.address_b(cpu_a[12:0]),
	.data_b(8'h00),
	.wren_b(1'b0),
	.q_b(extram_do)
);

// system 2k RAM at $0000-$07FF

wire [7:0] ram_do;
iecdrv_mem #(.DATAWIDTH(8), .ADDRWIDTH(11), .WRITE_B(0)) ram
(
	.clock_a(clk),
	.address_a(cpu_a[10:0]),
	.data_a(cpu_do),
	.wren_a(ena_r & ~cpu_rw & ram_cs),

	// See the note at extram: port B is read-only, so tie its write inputs off.
	.clock_b(clk),
	.address_b(cpu_a[10:0]),
	.data_b(8'h00),
	.wren_b(1'b0),
	.q_b(ram_do)
);

// 8 bytes scratch RAM at $4010-$4017 (1571CR only)

// wire [7:0] scrram_do;
// iecdrv_mem #(8,3) scrram
// (
// 	.clock_a(clk),
// 	.address_a(cpu_a[2:0]),
// 	.data_a(cpu_do),
// 	.wren_a(ena_r & ~cpu_rw & scrram_cs),

// 	.clock_b(clk),
// 	.address_b(cpu_a[2:0]),
// 	.q_b(scrram_do)
// );

// VIA1 1571-U9 (6522) signals

wire [7:0] via1_do;
wire [7:0] via1_cpu_do;
wire       via1_irq;
wire [7:0] via1_pa_o;
wire [7:0] via1_pa_oe;
wire       via1_ca2_o;
wire       via1_ca2_oe;
wire [7:0] via1_pb_o;
wire [7:0] via1_pb_oe;
wire [7:0] via1_pb_i = {~iec_atn_in, 2'(DRIVE), 2'b11, ~iec_clk_in, 1'b1, ~iec_data_in}
                       & (via1_pb_o | ~via1_pb_oe);
wire       via1_cb1_o;
wire       via1_cb1_oe;
wire       via1_cb2_o;
wire       via1_cb2_oe;

// XSim clears IFR bit 6 on a T1-high write as the 6522 specifies, but the Vivado
// implementation can retain a previously set bit: iecdrv_via6522 assigns both the
// complete irq_flags vector and its aliased timer_a_flag bit in one clocked process.
// Keep that proven, vendored VIA untouched and suppress only the stale polled value
// for the interval just loaded into T1. The real flag becomes visible again when the
// programmed one-shot expires, so a genuine EOI timeout still reaches $E9F2.
// Masking is confined to bit 6 of the polled value: the DOS enables only CA1 in the
// IER, so T1 never feeds irq_out and bit 7 still comes straight from the VIA.
wire       via1_t1h_wr = ena_f & ~cpu_rw & via1_cs & (cpu_a[3:0] == 4'h5);
wire       via1_t1l_wr = ena_f & ~cpu_rw & via1_cs &
                         ((cpu_a[3:0] == 4'h4) | (cpu_a[3:0] == 4'h6));
reg  [7:0] via1_t1_latch_low = 0;
reg [16:0] via1_t1_guard = 0;

always @(posedge clk) begin
	if (reset) begin
		via1_t1_latch_low <= 0;
		via1_t1_guard     <= 0;
	end
	else begin
		if (via1_t1l_wr) via1_t1_latch_low <= cpu_do;
		if (via1_t1h_wr)
			via1_t1_guard <= {1'b0, cpu_do, via1_t1_latch_low} + 1'd1;
		else if (ena_f && |via1_t1_guard)
			via1_t1_guard <= via1_t1_guard - 1'd1;
	end
end

assign via1_cpu_do = (cpu_rw && cpu_a[3:0] == 4'hD && |via1_t1_guard)
                   ? (via1_do & 8'hBF) : via1_do;

wire       fser_dir     = (via1_pa_o[1] | ~via1_pa_oe[1]) & |drv_mode;
assign     side         = (via1_pa_o[2] | ~via1_pa_oe[2]) &  drv_mode[1];
wire       soe;
wire       via_accl_ctl = (via1_pa_o[5] | ~via1_pa_oe[5]) & |drv_mode;
// Match the original 1571: VIA1 PA5 is the sole 1/2 MHz selector. The DOS keeps
// it low in 1541 compatibility mode and raises it in native 1571 mode.
wire       accl_ctl     = via_accl_ctl;

assign     iec_data_out = ~(via1_pb_o[1] | ~via1_pb_oe[1]) & ~((via1_pb_o[4] | ~via1_pb_oe[4]) ^ ~iec_atn_in) & (~fser_dir | cia_sp_out);
assign     iec_clk_out  = ~(via1_pb_o[3] | ~via1_pb_oe[3]);

assign     par_stb_out  = |drv_mode ?  cia_pc_n               : (via1_ca2_o | ~via1_ca2_oe);
assign     par_data_out = |drv_mode ? (cia_pb_o | ~cia_pb_oe) : (via1_pa_o  | ~via1_pa_oe);

iecdrv_via6522 via1
(
	.clock(clk),
	.rising(ena_r),
	.falling(ena_f),
	.reset(reset),

	.addr(cpu_a[3:0]),
	.wen(~cpu_rw & via1_cs),
	.ren(cpu_rw & via1_cs),
	.data_in(cpu_do),
	.data_out(via1_do),

	.port_a_o(via1_pa_o),
	.port_a_t(via1_pa_oe),                     
	.port_a_i(ext_en & ~|drv_mode ? par_data_in : {byte_n | ~|drv_mode, 6'h3F, ~tr00_sense} & (via1_pa_o | ~via1_pa_oe)),

	.port_b_o(via1_pb_o),
	.port_b_t(via1_pb_oe),
	.port_b_i(via1_pb_i),

	.ca1_i(~iec_atn_in),

	.ca2_o(via1_ca2_o),
	.ca2_t(via1_ca2_oe),
	.ca2_i(wps_n & (via1_ca2_o | ~via1_ca2_oe)),

	.cb1_o(via1_cb1_o),
	.cb1_t(via1_cb1_oe),
	.cb1_i(((ext_en & ~&drv_mode) ? par_stb_in : 1'b1) & (via1_cb1_o | ~via1_cb1_oe)),

	.cb2_o(via1_cb2_o),
	.cb2_t(via1_cb2_oe),
	.cb2_i(via1_cb2_o | ~via1_cb2_oe),

	.irq(via1_irq)
);

// VIA2 1571-U4 (6522) signals

wire [7:0] via2_do;
wire       via2_irq;
wire [7:0] via2_pa_o;
wire [7:0] via2_pa_oe;
wire       via2_ca2_o;
wire       via2_ca2_oe;
wire [7:0] via2_pb_o;
wire [7:0] via2_pb_oe;
wire       via2_cb1_o;
wire       via2_cb1_oe;
wire       via2_cb2_o;
wire       via2_cb2_oe;

wire       ted    = via2_cs    | ~accl[2];
assign     soe    = via2_ca2_o | ~via2_ca2_oe;
wire [7:0] gcr_di = via2_pa_o  | ~via2_pa_oe;
assign sector_gcr_din = gcr_di;
assign gcr_do = sector_gcr_enable ? sector_gcr_dout : h156_do;
assign sync_n = sector_gcr_enable ? sector_gcr_sync_n : h156_sync_n;
// c157x_h156 applies SOE internally, the sector-image GCR module does not. Without this
// gate the CPU sees byte-ready pulses while DOS has byte-ready disabled, exactly as in
// c1541_logic.sv of the reference 1541, where cpu_so_n = byte_n | ~soe.
assign byte_n = sector_gcr_enable ? (sector_gcr_byte_n | ~soe) : h156_byte_n;

assign     stp    = via2_pb_o[1:0] | ~via2_pb_oe[1:0];
assign     mtr    = via2_pb_o[2]   | ~via2_pb_oe[2];
assign     act    = via2_pb_o[3]   | ~via2_pb_oe[3];
assign     freq   = via2_pb_o[6:5] | ~via2_pb_oe[6:5];
assign     mode   = via2_cb2_o     | ~via2_cb2_oe;

iecdrv_via6522 via2
(
	.clock(clk),
	.rising(ena_r),
	.falling(ena_f),
	.reset(reset),

	.addr(cpu_a[3:0]),
	.wen(~cpu_rw & via2_cs),
	.ren(cpu_rw & via2_cs),
	.data_in(cpu_do),
	.data_out(via2_do),

	.port_a_o(via2_pa_o),
	.port_a_t(via2_pa_oe),
	.port_a_i(gcr_do & gcr_di),

	.port_b_o(via2_pb_o),
	.port_b_t(via2_pb_oe),
	.port_b_i({sync_n, 2'b11, wps_n, 4'b1111} & (via2_pb_o | ~via2_pb_oe)),

	.ca1_i(byte_n),

	.ca2_o(via2_ca2_o),
	.ca2_t(via2_ca2_oe),
	.ca2_i(via2_ca2_o | ~via2_ca2_oe),

	.cb1_o(via2_cb1_o),
	.cb1_t(via2_cb1_oe),
	.cb1_i(via2_cb1_o | ~via2_cb1_oe),

	.cb2_o(via2_cb2_o),
	.cb2_t(via2_cb2_oe),
	.cb2_i(via2_cb2_o | ~via2_cb2_oe),

	.irq(via2_irq)
);

// CIA 1571-U20 (6526/5710) signals

wire [7:0] cia_do;
wire       cia_irq_n;
wire [7:0] cia_pa_o, cia_pa_oe, cia_pb_o, cia_pb_oe;
wire       cia_pc_n;

wire       cia_sp_out;
wire       cia_cnt_out;

assign     iec_fclk_out = ~fser_dir | cia_cnt_out;

mos6526_8520 cia
(
	.res_n(~reset & |drv_mode),
	.clk(clk),
	.mode(&drv_mode ? 2'b11 : 2'b00),
	.phi2_p(ena_f),
	.phi2_n(ena_r),
	.cs_n(~cia_cs),
	.rw(cpu_rw),

	.rs(cpu_a[3:0]),
	.db_in(cpu_do),
	.db_out(cia_do),

	.pa_out(cia_pa_o),
	.pa_oe(cia_pa_oe),
	.pa_in(cia_pa_o | ~cia_pa_oe),

	.pb_out(cia_pb_o),
	.pb_oe(cia_pb_oe),
	.pb_in((ext_en ? par_data_in : 8'hff) & (cia_pb_o | ~cia_pb_oe)),

	.pc_n(cia_pc_n),

	.flag_n(ext_en ? par_stb_in : 1'b1),

	.tod(1'b1),

	.sp_in(fser_dir | iec_data_in),
	.sp_out(cia_sp_out),

	.cnt_in(fser_dir | iec_fclk_in),
	.cnt_out(cia_cnt_out),

	.irq_n(cia_irq_n)
);

// Head signals mux

assign     ht = gcr_ht | (mfm_ht & |drv_mode);
assign     hinit = gcr_hinit | (mfm_hinit & |drv_mode);

// 64H156 1571-U6 signals

c157x_h156 c157x_h156
(
	.clk(clk),
	.reset(reset),
	.enable(drive_enable & ~sector_gcr_enable),
	.mhz1_2(accl[1]),
	
	.hinit(gcr_hinit),
	.hclk(hclk),
	.ht(gcr_ht),
	.hf(hf),

	.mode(mode),
	.soe(soe),
	.ted(ted),
	.sync_n(h156_sync_n),
	.byte_n(h156_byte_n),

	.dout(h156_do),
	.din(gcr_di)
);

// FDC 1571-U11 (WD1770) signals

wire [7:0] wd_do;
wire       mfm_ht, mfm_hinit;
wire       mfm_wgate;

assign     wgate = mfm_wgate & |drv_mode;

c157x_fdc1772 #(.MODEL(0)) c157x_fdc1772
(
	.clkcpu(clk),
	.clk8m_en(wd_ce),

	.floppy_reset(~reset & |drv_mode),
	.floppy_present(disk_present),
	// .floppy_side(side),
	.floppy_motor(mtr),
	.floppy_index(~index_sense),
	.floppy_wprot(~(wps_n & img_mfm)),
	.floppy_track00(1),

	.hinit(mfm_hinit),
	.hclk(hclk),
	.ht(mfm_ht),
	.hf(hf),
	.wgate(mfm_wgate),
	.busy(fdc_busy),

	.cpu_addr(cpu_a[1:0]),
	.cpu_sel(wd_cs),
	.cpu_rw(cpu_rw | ~ena_r),
	.cpu_din(cpu_do),
	.cpu_dout(wd_do)
);

// MEGA65 diagnostics: what the DOS itself is doing, which is invisible from the host
// side of the drive. Everything here only observes, so it can stay in a release build.

// Zero page mirrors. The 1541/1571 DOS keeps its disk controller handshake here: $00-$05
// are the six job slots (bit 7 set means "job requested"), $20 is the drive state and
// $48 the motor spin-up countdown. Reading them tells us whether the serial side ever
// handed a job to the disk side, which is the split the LED cannot resolve.
reg [7:0] zp_job0 = 0;
reg [7:0] job_posts = 0;
reg [7:0] zp_bit_count = 0;

wire zp_wr = ena_r & ~cpu_rw & ram_cs & (cpu_a[15:8] == 8'h00);

always @(posedge clk) begin
	if (reset) begin
		zp_job0   <= 0;
		zp_state  <= 0;
		job_posts <= 0;
		zp_bit_count <= 0;
	end
	else if (zp_wr) begin
		case (cpu_a[7:0])
			8'h00: zp_job0   <= cpu_do;
			8'h20: zp_state  <= cpu_do;
			8'h98: zp_bit_count <= cpu_do;
		endcase
		if (cpu_a[7:0] < 8'h06 && cpu_do[7]) job_posts <= job_posts + 1'd1;
	end
end

// CPU side of the serial bus. orb_writes counts stores to VIA1 ORB ($1800), which is the
// only way the DOS can drive CLK, DATA and ATNA, and irq_taken counts fetches of the high
// byte of the IRQ vector. Together they separate "the DOS never runs its ATN service" from
// "the DOS runs it but the handshake goes wrong".
wire via1_orb_wr = ena_r & ~cpu_rw & via1_cs & (cpu_a[3:0] == 4'h0);
wire via1_orb_rd = ena_r &  cpu_rw & via1_cs & (cpu_a[3:0] == 4'h0);
wire via1_ifr_rd = ena_r &  cpu_rw & via1_cs & (cpu_a[3:0] == 4'hD);
wire irq_fetch   = ena_r &  cpu_rw & (cpu_a[15:0] == 16'hFFFF);

reg [7:0] orb_writes = 0;
reg [7:0] irq_taken = 0;
reg [7:0] t1h_writes = 0;
reg [7:0] last_t1h = 0;
reg [7:0] eoi_entries = 0;
reg [7:0] byte_acks = 0;
reg [7:0] last_acr = 0;
reg [7:0] last_ier = 0;
reg [7:0] last_t1l = 0;
reg [7:0] last_t1lh = 0;
reg [7:0] t1lh_writes = 0;

always @(posedge clk) begin
	if (reset) begin
		orb_writes <= 0;
		irq_taken  <= 0;
		t1h_writes <= 0;
		last_t1h   <= 0;
		eoi_entries <= 0;
		byte_acks   <= 0;
		last_acr    <= 0;
		last_ier    <= 0;
		last_t1l    <= 0;
		last_t1lh   <= 0;
		t1lh_writes <= 0;
	end
	else begin
		if (via1_orb_wr) orb_writes <= orb_writes + 1'd1;
		if (irq_fetch)   irq_taken  <= irq_taken + 1'd1;
		// Sample on ena_f: this is the phase the VIA itself uses for writes.
		if (via1_t1h_wr) begin
			t1h_writes <= t1h_writes + 1'd1;
			last_t1h   <= cpu_do;
		end
		// These ROM addresses distinguish the two paths that both call $E9A5 and
		// therefore produce indistinguishable DATA-low pulses on the IEC bus.
		if (ena_f && cpu_a[15:0] == 16'hE9F2) eoi_entries <= eoi_entries + 1'd1;
		if (ena_f && cpu_a[15:0] == 16'hEA28) byte_acks   <= byte_acks + 1'd1;
		// ACR bit 6 decides whether T1 re-fires by itself and bit 7 steals PB7, which is
		// the ATN input on VIA1; both change what a set T1 flag means.
		if (ena_f & ~cpu_rw & via1_cs & (cpu_a[3:0] == 4'hB)) last_acr <= cpu_do;
		if (ena_f & ~cpu_rw & via1_cs & (cpu_a[3:0] == 4'hE)) last_ier <= cpu_do;
		// A T1 reload takes its period from the latch, not from the counter write, so a
		// short latch makes the flag come back microseconds after every clear. $1804 and
		// $1806 both load the low latch; $1807 loads the high latch without restarting.
		if (ena_f & ~cpu_rw & via1_cs & ((cpu_a[3:0] == 4'h4) | (cpu_a[3:0] == 4'h6)))
			last_t1l <= cpu_do;
		if (ena_f & ~cpu_rw & via1_cs & (cpu_a[3:0] == 4'h7)) begin
			last_t1lh   <= cpu_do;
			t1lh_writes <= t1lh_writes + 1'd1;
		end
	end
end

// The whole ATN handshake is over in about a millisecond, far below the rate at which the
// shell samples this bank, so a plain mirror of the bus always reads back idle. atn_acc
// therefore accumulates sticky bits for the duration of one ATN-low window and is cleared
// only by the next falling edge, so a slow reader still sees what happened during the last
// one. atn_pb_o/atn_pb_oe follow VIA1 port B while ATN is low and so hold the state the
// port had when the window ended.
reg [7:0]  atn_edges = 0;
reg        atn_d = 1;
reg [15:0] atn_acc = 0;
reg [7:0]  atn_pb_o = 0;
reg [7:0]  atn_pb_oe = 0;
reg [15:0] atn_ticks = 0;

always @(posedge clk) begin
	atn_d <= iec_atn_in;
	if (reset) begin
		atn_edges <= 0;
		atn_acc   <= 0;
		atn_pb_o  <= 0;
		atn_pb_oe <= 0;
		atn_ticks <= 0;
	end
	else if (atn_d & ~iec_atn_in) begin
		atn_edges <= atn_edges + 1'd1;
		atn_acc   <= 0;
		atn_ticks <= 0;
	end
	else if (~iec_atn_in) begin
		atn_acc[0]  <= atn_acc[0]  | ~iec_data_out;   // drive pulls DATA (ATN acknowledge)
		atn_acc[1]  <= atn_acc[1]  | ~iec_clk_out;    // drive pulls CLK
		atn_acc[2]  <= atn_acc[2]  | ~iec_data_in;    // DATA low on the merged bus
		atn_acc[3]  <= atn_acc[3]  | ~iec_clk_in;     // CLK low on the merged bus
		atn_acc[4]  <= atn_acc[4]  |  via1_irq;
		atn_acc[5]  <= atn_acc[5]  |  via2_irq;
		atn_acc[6]  <= atn_acc[6]  | ~cia_irq_n;
		atn_acc[7]  <= atn_acc[7]  | ~cpu_irq_n;
		atn_acc[8]  <= atn_acc[8]  |  via1_orb_wr;
		atn_acc[9]  <= atn_acc[9]  |  via1_orb_rd;
		atn_acc[10] <= atn_acc[10] |  irq_fetch;
		atn_acc[11] <= atn_acc[11] |  fser_dir;
		atn_acc[12] <= atn_acc[12] |  accl[1];        // CPU ran at 2 MHz during the window
		atn_acc[13] <= atn_acc[13] |  halt;
		atn_acc[14] <= atn_acc[14] | &atn_ticks;      // window longer than the tick counter
		atn_acc[15] <= 1'b1;                          // a window was observed at all
		atn_pb_o    <= via1_pb_o;
		atn_pb_oe   <= via1_pb_oe;
		if (~&atn_ticks) atn_ticks <= atn_ticks + 1'd1;
	end
end

// The sticky bits above prove the drive answers ATN, but they collapse the whole window
// into one value and so cannot show where inside the command byte the handshake stalls.
// The trace records one entry per change of the serial bus, timestamped in microseconds
// since the previous entry, and restarts on every ATN falling edge: the buffer therefore
// always holds the beginning of the most recent attempt without needing to be armed.
localparam TRACE_LEN = 48;

wire [7:0] trace_state = {iec_atn_in, iec_clk_in, iec_data_in, iec_data_out,
                          iec_clk_out, via1_pb_o[4], via1_pb_o[1], via1_pb_o[3]};

reg [15:0] trace[TRACE_LEN];
reg [15:0] dos_trace[TRACE_LEN];
reg  [5:0] trace_idx  = 0;
reg  [7:0] trace_age  = 0;   // microseconds since the entry before, saturating
reg  [7:0] trace_last = 0;
reg  [7:0] last_orb   = 0;   // last value LDA $1800 actually returned
reg  [7:0] last_ifr   = 0;   // last value LDA $180D actually returned

// The DOS decides between "receive the next bit" and "acknowledge EOI" purely on the T1
// flag it reads out of $180D, so mirroring the value the CPU was handed says which way it
// branched without having to instrument the VIA itself.
wire [15:0] dos_state = {zp_bit_count, last_ifr};

always @(posedge clk) begin
	if (reset) begin
		trace_idx  <= 0;
		trace_age  <= 0;
		trace_last <= trace_state;
		last_orb   <= 0;
		last_ifr   <= 0;
		for (int i = 0; i < TRACE_LEN; i++) begin
			trace[i]     <= 0;
			dos_trace[i] <= 0;
		end
	end
	else begin
		if (via1_orb_rd) last_orb <= via1_do;
		if (via1_ifr_rd) last_ifr <= via1_cpu_do;

		if (atn_d & ~iec_atn_in) begin
			trace[0]     <= {trace_state, 8'h00};
			dos_trace[0] <= dos_state;
			trace_idx  <= 1;
			trace_age  <= 0;
			trace_last <= trace_state;
		end
		else begin
			// ph2_r[0] is the drive's 1 MHz phase, so it doubles as the microsecond tick.
			if (ph2_r[0] & ~&trace_age) trace_age <= trace_age + 1'd1;

			// A saturated age still writes an entry, so a long wait stays visible as a run
			// of 255 us gaps instead of silently folding into the next transition.
			if (|trace_idx && trace_idx < TRACE_LEN &&
			    (trace_state != trace_last || &trace_age)) begin
				trace[trace_idx]     <= {trace_state, trace_age};
				dos_trace[trace_idx] <= dos_state;
				trace_idx        <= trace_idx + 1'd1;
				trace_age        <= 0;
				trace_last       <= trace_state;
			end
		end
	end
end

// Every timestamp above is counted in ph2_r[0], so a drive whose time base is wrong
// produces a trace that is internally consistent and still disagrees with the C128, which
// is the real-time master. Counting those same ticks against the main core clock gives an
// independent reference: at 31.5 MHz a correct 1 MHz phase yields ~1040 ticks per 32768
// clocks, and a drive running at the wrong rate shows up directly in this number.
reg [14:0] rate_window = 0;
reg [11:0] rate_count  = 0;
reg [11:0] ph2_rate    = 0;

always @(posedge clk) begin
	if (reset) begin
		rate_window <= 0;
		rate_count  <= 0;
		ph2_rate    <= 0;
	end
	else begin
		rate_window <= rate_window + 1'd1;
		if (&rate_window) begin
			ph2_rate   <= ph2_r[0] ? rate_count + 1'd1 : rate_count;
			rate_count <= 0;
			ph2f_rate  <= ena_f ? f_count + 1'd1 : f_count;
			f_count    <= 0;
		end
		else begin
			if (ph2_r[0]) rate_count <= rate_count + 1'd1;
			if (ena_f)    f_count    <= f_count + 1'd1;
		end
	end
end

// Timer 1 counts on ena_f, not on ph2_r[0], so the tick counter above does not actually
// cover the enable the timer runs from. Counting that one too closes the gap.
reg [11:0] f_count   = 0;
reg [11:0] ph2f_rate = 0;

// The decisive measurement for the spurious EOI: writing $1805 is supposed to clear the
// T1 interrupt flag, so the first $180D the DOS reads afterwards must come back with bit 6
// low until the timer genuinely runs out, which for a $01xx load is at least 256 us. Age
// is in microseconds since the arming write and saturates, so a value of a few us means
// the write never cleared the flag, while ~256+ means the timer really did expire.
reg [7:0] t1_arm_age   = 8'hFF;
reg [7:0] t1_flag_age  = 8'hFF;
reg [7:0] t1_first_ifr = 0;
reg       t1_armed     = 0;
reg [7:0] ifr_reads    = 0;

always @(posedge clk) begin
	if (reset) begin
		t1_arm_age   <= 8'hFF;
		t1_flag_age  <= 8'hFF;
		t1_first_ifr <= 0;
		t1_armed     <= 0;
		ifr_reads    <= 0;
	end
	else begin
		if (via1_t1h_wr) begin
			t1_arm_age <= 0;
			t1_armed   <= 1;
		end
		else if (ph2_r[0] & ~&t1_arm_age) t1_arm_age <= t1_arm_age + 1'd1;

		if (via1_ifr_rd) begin
			ifr_reads <= ifr_reads + 1'd1;
			if (t1_armed) begin
				t1_first_ifr <= via1_do;
				t1_flag_age  <= t1_arm_age;
				t1_armed     <= 0;
			end
		end
	end
end

genvar t;
generate
	for (t = 0; t < TRACE_LEN; t = t + 1) begin :trace_words
		assign dos_diag[128 + t*16 +: 16] = trace[t];
		assign dos_diag[896 + t*16 +: 16] = dos_trace[t];
	end
endgenerate

assign dos_diag[1679:1664] = cpu_a[15:0];          // live CPU bus address
assign dos_diag[1695:1680] = {cpu_di, cpu_do};     // live CPU input/output data
assign dos_diag[1711:1696] = {8'b0, cpu_rw, cpu_irq_n, via1_irq, via2_irq,
                              cia_irq_n, ena_r, ena_f, halt};
assign dos_diag[1727:1712] = dos_state;            // live $98 bit counter and last $180D
assign dos_diag[1743:1728] = {last_orb, via1_pb_i};// last $1800 read vs the raw port B pins
assign dos_diag[1759:1744] = {4'b0, ph2_rate};     // drive ticks per 32768 core clocks
assign dos_diag[1775:1760] = {last_t1h, t1h_writes};
assign dos_diag[1791:1776] = {eoi_entries, byte_acks};
assign dos_diag[1807:1792] = {t1_first_ifr, t1_flag_age};
assign dos_diag[1823:1808] = {last_acr, last_ier};
assign dos_diag[1839:1824] = {4'b0, ph2f_rate};
assign dos_diag[1855:1840] = {ifr_reads, t1lh_writes};
assign dos_diag[1871:1856] = {last_t1lh, last_t1l};
assign dos_diag[1919:1872] = 0;

// Word order matches the order the shell prints them, lowest word first.
assign dos_diag[15:0]    = 16'hD05A;               // signature
assign dos_diag[31:16]   = atn_acc;                // sticky state of the last ATN-low window
assign dos_diag[47:32]   = {atn_pb_o, atn_pb_oe};  // VIA1 port B at the end of that window
assign dos_diag[63:48]   = {zp_state, zp_job0};
assign dos_diag[79:64]   = {irq_taken, job_posts};
assign dos_diag[95:80]   = {orb_writes, atn_edges};
assign dos_diag[111:96]  = {via1_pb_o, via1_pb_oe};
assign dos_diag[127:112] = {4'b0000, iec_atn_in, iec_clk_in, iec_data_in, iec_fclk_in,
                            4'b0000, iec_clk_out, iec_data_out, mtr, drive_enable};

endmodule
