//--------------------------------------------------------------------------//
// tb_ap040_pipe_prog.v - program bench for the pipelined core (Minimig     //
// plan M1+)                                                                //
//                                                                          //
//   vvp tb.vvp +prog=<image.hex> +expect=<file.exp> [+cycles=N]            //
//                                                                          //
// Loads a flat image (one 16-bit word per line from address 0, bin2hex.py  //
// format) into the L1 array, runs the core from reset with                 //
// RESET_FROM_VECTORS=1 (ISP/PC from vectors 0/1), stops when the halt PC   //
// retires (or after +cycles clocks, default 20000), then checks every line //
// of the expect file:                                                      //
//   halt <pc>            stop when the instruction at <pc> retires         //
//   berr <addr> <r|w|f>  (bus mode) the first read/write/fetch touching    //
//                        <addr> bus-errors; a "w" address is never posted  //
//   lberr <addr>         (bus mode) the first write touching <addr>        //
//                        bus-errors AFTER it was posted (-DSTORE_BUF)      //
//                        (required; the first line normally)               //
//   d0..d7 a0..a6 <v>    register value (hex)                              //
//   usp|isp|msp <v>                                                        //
//   sr <v>  ccr <v>      (ccr: the low 5 bits)                             //
//   vbr <v>                                                                //
//   m8|m16|m32 <addr> <v>  memory (big-endian)                             //
//   noread <addr>        no data read may touch this byte (pure writes)     //
//   fpiar <v>            the FPU's FPIAR, read hierarchically (-DFPU_REAL)  //
//   x80 <addr> <se> <mant> <n>  an extended value (sign/exponent word,   //
//                        64-bit mantissa) within n units in the last place //
//                        (the FPSP's transcendentals: within 1 ulp, plan M10)//
//   # ...                comment (the # a word of its own)                 //
// Prints "ALL TESTS PASSED" or one FAIL line per mismatch.                  //
//                                                                          //
// -DBUS_MODE (plan M5): the core with BUS=1 and a memory model on its mem_* //
// port instead of the L1 array; +prof=0 back-to-back acks, 1 varied waits,  //
// 2 varied waits under a clock enable high one clock in three with the ack //
// held until the next enable (the Minimig kernel's clkena).               //
// -DSTORE_BUF=<n> builds the core with that store buffer mode             //
// (findings/storebuf/plan.md); the bench's RAM window is everything but   //
// $xxxxF000-$xxxxFFFF and the "berr ... w" addresses.  In bus mode         //
// tb_sb_check.v checks the store order on every run.                      //
//--------------------------------------------------------------------------//
`timescale 1ns/1ps

module tb_ap040_pipe_prog;

parameter [31:0] PC_RESET = 32'h0000_0400;
parameter         L1_AW   = 15;     // 64K: the whole image (t_integer uses $3000-$3400 and $F100);
                                    // the memory model (-DBUS_MODE) follows it, and the FPSP leg sets 17
localparam        L1_WORDS = (1 << L1_AW);

reg clk = 0;
always #5 clk = ~clk;
reg nreset = 0;
reg ce = 1'b1;
integer prof = 0;

wire        dbg_wb_valid;
wire [31:0] dbg_wb_pc;
wire [31:0] dbg_d0, dbg_d1, dbg_d2, dbg_d3, dbg_d4, dbg_d5, dbg_d6, dbg_d7;
wire  [4:0] dbg_ccr;
wire [15:0] dbg_sr;

wire        mem_req, mem_write, mem_instr;
wire  [1:0] mem_size;
wire [31:0] mem_addr, mem_wdata;
wire  [2:0] mem_fc;
reg         m_ack = 1'b0;
wire [31:0] stw_addr;             // (the store buffer's RAM window, below)
reg         stw_ram;
reg         m_flt = 1'b0;         // (bus mode) the answer is an access error
reg  [31:0] m_rdata = 32'd0;

// (plan M10.1) the FPU port group: with -DFPU_STUB the core is built with
// HAS_FPU = 1 and tb_fpu_stub.v answers it, so the req/accepted/done
// interlock can be exercised by a program before the real unit goes in.
// With -DFPU_REAL the lifted ap040_fpu answers instead, wired by the SAME
// include the wrapper uses (rtl/compat/ap040_fpu_tie.vh), so a program that
// passes here drives the instance the bitstream carries.
wire        fp_req, fp_done, fp_accepted, fp_unimp, fp_unsupp, fp_dbl;
wire  [2:0] fp_op_class, fp_src_fmt, fp_src_r, fp_dst_r;
wire  [6:0] fp_opmode;
wire [95:0] fp_din, fp_dout;
wire        fp_ia_we, fp_cr_we, fp_exc_req;
wire  [7:0] fp_exc_vec;
wire [31:0] fp_ia_wdata, fp_cr_wdata, fp_cr_rdata;
wire  [1:0] fp_cr_sel;
wire        fp_fm_we;
wire  [2:0] fp_fm_sel;
wire [95:0] fp_fm_wdata, fp_fm_rdata;
wire        fp_used, fp_frst, fp_fridle, fp_bsun_en, fp_bsun_req;
wire  [3:0] fp_fpcc;
wire        fp_st_unimp, fp_st_wbte15, fp_fsave_ack, fp_fr_unimp, fp_fr_wbte15;
wire [15:0] fp_st_cmd1, fp_st_cmd3, fp_fr_cmd1, fp_fr_cmd3;
wire  [2:0] fp_st_stag, fp_st_dtag, fp_st_flags, fp_st_grs;
wire  [2:0] fp_fr_stag, fp_fr_dtag, fp_fr_flags, fp_fr_grs;
wire [95:0] fp_st_fpt, fp_st_et, fp_fr_fpt, fp_fr_et;
wire        fp_st_busy, fp_fr_busy, fp_fr_et15, fp_fr_fpt15, fp_fr_resume;
wire [95:0] fp_st_wbt, fp_fr_wbt;
wire [31:0] fp_st_fpiar, fp_fr_fpiar;
wire  [7:0] fp_fr_cusavepc;
wire        fp_pcap, fp_fr_e1pend;
wire  [7:0] fp_cur_vec;
wire        fp_ce = ce;
`ifdef FPU_REAL
`ifndef XC7
`define XC7 0
`endif
localparam AP040_XC7 = `XC7;   // the wrapper's switch, for the include
`include "ap040_fpu_tie.vh"
assign fp_dbl = 1'b0;
`elsif FPU_STUB
tb_fpu_stub u_fpu
(
	.clk(clk), .nreset(nreset), .ce(ce),
	.req(fp_req), .op_class(fp_op_class), .opmode(fp_opmode),
	.src_fmt(fp_src_fmt), .src_r(fp_src_r), .dst_r(fp_dst_r), .din(fp_din),
	.done(fp_done), .accepted(fp_accepted), .unimp(fp_unimp), .unsupp(fp_unsupp),
	.dout(fp_dout), .dbl_req(fp_dbl)
);
`else
assign fp_done = 1'b0, fp_accepted = 1'b0, fp_unimp = 1'b0, fp_unsupp = 1'b0;
assign fp_dout = 96'd0, fp_dbl = 1'b0;
`endif
`ifndef FPU_REAL
// only the real unit has control registers or arithmetic exceptions; without
// it these would be undriven wires, which is X and would poison the decode
assign fp_cr_rdata = 32'd0;
assign fp_exc_req = 1'b0, fp_exc_vec = 8'd0;
assign fp_fm_rdata = 96'd0;
assign fp_used = 1'b0;
assign fp_fpcc = 4'd0, fp_bsun_en = 1'b0;
assign fp_st_unimp = 1'b0, fp_st_wbte15 = 1'b0;
assign fp_st_cmd1 = 16'd0, fp_st_cmd3 = 16'd0;
assign fp_st_stag = 3'd0, fp_st_dtag = 3'd0, fp_st_flags = 3'd0, fp_st_grs = 3'd0;
assign fp_st_fpt = 96'd0, fp_st_et = 96'd0;
assign fp_st_busy = 1'b0, fp_st_wbt = 96'd0, fp_st_fpiar = 32'd0, fp_fr_resume = 1'b0;
assign fp_fr_e1pend = 1'b0, fp_cur_vec = 8'd0;
`endif

ap040_pipe_core #(
	.PC_RESET(PC_RESET), .PROG_WORDS(32'h7FFF_FFFF), .L1_AW(L1_AW), .RESET_FROM_VECTORS(1)
`ifdef BUS_MODE
	, .BUS(1)
`endif
`ifdef STORE_BUF
	, .STORE_BUF(`STORE_BUF)
`endif
`ifdef BTB
	, .BTB(`BTB)
`endif
`ifdef LDX
	, .LDX(`LDX)
`endif
`ifdef BTFN
	, .BTFN(`BTFN)
`endif
`ifdef DFP_MIS
	, .DFP_MIS(`DFP_MIS)
`endif
`ifdef RAS
	, .RAS(`RAS)
`endif
`ifdef MISPLIT
	, .MISPLIT(`MISPLIT)
`endif
`ifdef FPU_STUB
	, .HAS_FPU(1)
`endif
`ifdef FPU_REAL
	, .HAS_FPU(1)
`endif
) dut (
	.clk(clk), .nreset(nreset), .ce(ce),
	.mem_req(mem_req), .mem_write(mem_write), .mem_instr(mem_instr), .mem_size(mem_size),
	.mem_addr(mem_addr), .mem_wdata(mem_wdata), .mem_fc(mem_fc),
	.mem_ack(m_ack), .mem_rdata(m_rdata), .mem_flt(m_flt), .mem_atc(1'b0), .bus_st_err(),
	.stw_addr(stw_addr), .stw_ram(stw_ram), .mem_postok(1'b0),
	.dbg_if_valid(), .dbg_if_pc(), .dbg_id_valid(), .dbg_id_pc(),
	.dbg_eac_valid(), .dbg_eac_pc(), .dbg_eaf_valid(), .dbg_eaf_pc(),
	.dbg_ex_valid(), .dbg_ex_pc(), .dbg_wb_valid(dbg_wb_valid), .dbg_wb_pc(dbg_wb_pc),
	.dbg_d0(dbg_d0), .dbg_d1(dbg_d1), .dbg_d2(dbg_d2), .dbg_d3(dbg_d3),
	.dbg_d4(dbg_d4), .dbg_d5(dbg_d5), .dbg_d6(dbg_d6), .dbg_d7(dbg_d7),
	.dbg_ccr(dbg_ccr), .dbg_sr(dbg_sr),
	.fp_req(fp_req), .fp_op_class(fp_op_class), .fp_opmode(fp_opmode),
	.fp_src_fmt(fp_src_fmt), .fp_src_r(fp_src_r), .fp_dst_r(fp_dst_r), .fp_din(fp_din),
	.fp_ia_we(fp_ia_we), .fp_ia_wdata(fp_ia_wdata),
	.fp_cr_sel(fp_cr_sel), .fp_cr_we(fp_cr_we), .fp_cr_wdata(fp_cr_wdata), .fp_cr_rdata(fp_cr_rdata),
	.fp_fm_sel(fp_fm_sel), .fp_fm_we(fp_fm_we), .fp_fm_wdata(fp_fm_wdata), .fp_fm_rdata(fp_fm_rdata),
	.fp_fpcc(fp_fpcc), .fp_bsun_en(fp_bsun_en), .fp_bsun_req(fp_bsun_req),
	.fp_used(fp_used), .fp_frst(fp_frst), .fp_fridle(fp_fridle),
	.fp_st_unimp(fp_st_unimp), .fp_st_cmd1(fp_st_cmd1), .fp_st_cmd3(fp_st_cmd3),
	.fp_st_stag(fp_st_stag), .fp_st_dtag(fp_st_dtag), .fp_st_flags(fp_st_flags),
	.fp_st_grs(fp_st_grs), .fp_st_wbte15(fp_st_wbte15),
	.fp_st_fpt(fp_st_fpt), .fp_st_et(fp_st_et),
	.fp_fsave_ack(fp_fsave_ack), .fp_fr_unimp(fp_fr_unimp),
	.fp_fr_cmd1(fp_fr_cmd1), .fp_fr_cmd3(fp_fr_cmd3),
	.fp_fr_stag(fp_fr_stag), .fp_fr_dtag(fp_fr_dtag), .fp_fr_flags(fp_fr_flags),
	.fp_fr_grs(fp_fr_grs), .fp_fr_wbte15(fp_fr_wbte15),
	.fp_fr_fpt(fp_fr_fpt), .fp_fr_et(fp_fr_et),
	.fp_st_busy(fp_st_busy), .fp_st_wbt(fp_st_wbt), .fp_st_fpiar(fp_st_fpiar),
	.fp_fr_busy(fp_fr_busy), .fp_fr_wbt(fp_fr_wbt), .fp_fr_fpiar(fp_fr_fpiar),
	.fp_fr_cusavepc(fp_fr_cusavepc), .fp_fr_et15(fp_fr_et15), .fp_fr_fpt15(fp_fr_fpt15),
	.fp_fr_resume(fp_fr_resume),
	.fp_pcap(fp_pcap), .fp_fr_e1pend(fp_fr_e1pend), .fp_cur_vec(fp_cur_vec),
	.fp_done(fp_done), .fp_accepted(fp_accepted), .fp_unimp(fp_unimp), .fp_unsupp(fp_unsupp),
	.fp_exc_req(fp_exc_req), .fp_exc_vec(fp_exc_vec),
	.fp_dout(fp_dout)
);

// "berr" lines (bus mode only)
reg [31:0] be_at [0:7];
reg  [7:0] be_kind [0:7];
reg        be_used [0:7];
integer    n_be = 0, kbe;

// the store buffer's RAM window: not the bench's I/O page, not a store
// the program expects a precise bus error on ("berr <a> w")
always @* begin
	stw_ram = (stw_addr[15:12] != 4'hF);
	for (kbe = 0; kbe < n_be; kbe = kbe + 1)
		if (be_kind[kbe] == "w" && be_at[kbe] + 32'd3 >= stw_addr && be_at[kbe] < stw_addr + 32'd4)
			stw_ram = 1'b0;
end
`ifdef BUS_MODE
reg [15:0] bmem [0:L1_WORDS-1];       // 64K at address 0 by default (high bits alias, as the L1)
function [7:0] mb(input [31:0] a);
	reg [15:0] w;
	begin
		w  = bmem[(a >> 1) & (L1_WORDS - 1)];
		mb = a[0] ? w[7:0] : w[15:8];
	end
endfunction
`else
function [7:0] mb(input [31:0] a);
	reg [15:0] w;
	begin
		w  = dut.u_l1.mem[((a - PC_RESET) >> 1) & (L1_WORDS - 1)];
		mb = a[0] ? w[7:0] : w[15:8];
	end
endfunction
`endif

// data accesses as the checks below see them: the L1's ports, or (bus mode)
// a completing mem_* transfer that is not a fetch
wire        ev_rd, ev_wr, ev_wr_rb;
wire [31:0] ev_rd_addr, ev_wr_addr, ev_wr_data;
wire  [1:0] ev_rd_size, ev_wr_size;
wire  [2:0] ev_rd_fc, ev_wr_fc;
`ifdef BUS_MODE
reg         m_busy = 1'b0, m_first = 1'b0;
integer     m_lat = 0, m_n = 0, cyc = 0;
// an access event is the clock after the memory answered (m_first): the
// request is still held then, and the write lands at the falling edge
assign ev_rd = m_first && !mem_write && !mem_instr;
assign ev_wr = m_first && mem_write;
assign ev_rd_addr = mem_addr; assign ev_wr_addr = mem_addr;
assign ev_rd_size = mem_size; assign ev_wr_size = mem_size;
assign ev_rd_fc   = mem_fc;   assign ev_wr_fc   = mem_fc;
assign ev_wr_data = mem_wdata;
assign ev_wr_rb   = dut.g_bus.u_bcu.mem_rb;
function integer mlat(input integer p, input integer n);
	mlat = (p == 0) ? 0 : (n * 7 + 3) % 6;
endfunction
function [31:0] mrd(input [31:0] a, input [1:0] sz);
	case (sz)
		2'd0:    mrd = {24'd0, mb(a)};
		2'd1:    mrd = {16'd0, mb(a), mb(a + 1)};
		default: mrd = {mb(a), mb(a + 1), mb(a + 2), mb(a + 3)};
	endcase
endfunction
task mwb(input [31:0] a, input [7:0] b);
	if (a[0]) bmem[(a >> 1) & (L1_WORDS - 1)][7:0] = b; else bmem[(a >> 1) & (L1_WORDS - 1)][15:8] = b;
endtask
// "berr <addr> <r|w|f>": the first data read / data write / fetch that
// touches <addr> ends in a bus error instead (one-shot per line), M6
function automatic be_hit(input [31:0] a, input [1:0] sz, input w, input ins);
	integer kk; reg h; reg [31:0] n;
	begin
		h = 1'b0; n = (sz == 2'd0) ? 1 : (sz == 2'd1) ? 2 : 4;
		for (kk = 0; kk < n_be; kk = kk + 1)
			if (!be_used[kk] && be_at[kk] >= a && be_at[kk] < a + n &&
			    ((be_kind[kk] == "r" && !w && !ins) || ((be_kind[kk] == "w" || be_kind[kk] == "l") && w) ||
			     (be_kind[kk] == "f" && ins)))
				h = 1'b1;
		be_hit = h;
	end
endfunction
task be_use(input [31:0] a, input [1:0] sz, input w, input ins);
	integer kk; reg [31:0] n;
	begin
		n = (sz == 2'd0) ? 1 : (sz == 2'd1) ? 2 : 4;
		for (kk = 0; kk < n_be; kk = kk + 1)
			if (!be_used[kk] && be_at[kk] >= a && be_at[kk] < a + n &&
			    ((be_kind[kk] == "r" && !w && !ins) || ((be_kind[kk] == "w" || be_kind[kk] == "l") && w) ||
			     (be_kind[kk] == "f" && ins)))
				be_used[kk] = 1'b1;
	end
endtask

// the memory: a request is taken when it appears, answered m_lat clocks
// later; the ack stays up until a clock-enabled edge has seen it
always @(posedge clk) begin
	cyc <= cyc + 1;
	if (!nreset) begin m_busy <= 1'b0; m_ack <= 1'b0; m_first <= 1'b0; end
	else begin
		m_first <= 1'b0;
		if (m_ack && ce) begin m_ack <= 1'b0; m_flt <= 1'b0; end
		if (!m_busy && mem_req && !m_ack) begin
			m_busy <= 1'b1; m_lat <= mlat(prof, m_n); m_n <= m_n + 1;
		end else if (m_busy) begin
			if (m_lat == 0 && be_hit(mem_addr, mem_size, mem_write, mem_instr)) begin
				// a bus error ends the transfer: the fault, no data, no write
				be_use(mem_addr, mem_size, mem_write, mem_instr);
				m_busy <= 1'b0; m_ack <= 1'b1; m_flt <= 1'b1;
				m_rdata <= 32'hDEAD_BEEF;
			end else if (m_lat == 0) begin
				m_busy <= 1'b0; m_ack <= 1'b1; m_first <= 1'b1;
				m_rdata <= mrd(mem_addr, mem_size);
			end else m_lat <= m_lat - 1;
		end
	end
end
// the request must be low for one enabled clock after an ack (the reference
// core's S_EPF_GAP rule: the cache and the RAM controllers take a held
// request after an ack as a new transfer)
reg     ack_seen = 1'b0;
integer gap_bad = 0;
always @(posedge clk)
	if (nreset && ce) begin
		if (ack_seen && mem_req) begin
			gap_bad = gap_bad + 1;
			if (gap_bad < 4) $display("FAIL: mem_req high in the enabled clock after an ack (addr %h)", mem_addr);
		end
		ack_seen <= m_ack;
	end
always @(negedge clk)
	if (m_first && mem_write)
		case (mem_size)
			2'd0: mwb(mem_addr, mem_wdata[7:0]);
			2'd1: begin mwb(mem_addr, mem_wdata[15:8]); mwb(mem_addr + 1, mem_wdata[7:0]); end
			default: begin
				mwb(mem_addr, mem_wdata[31:24]); mwb(mem_addr + 1, mem_wdata[23:16]);
				mwb(mem_addr + 2, mem_wdata[15:8]); mwb(mem_addr + 3, mem_wdata[7:0]);
			end
		endcase
always @(posedge clk) ce <= (prof == 2) ? ((cyc % 3) == 0) : 1'b1;

// the store order (tb_sb_check.v)
tb_sb_check u_sbc
(
	.clk(clk), .nreset(nreset), .ce(ce),
	.ret_v(dut.retire && dut.exe_o.st_v && !dut.wb_fault),
	.ret_a(dut.exe_o.st_addr), .ret_s(dut.exe_o.st_size), .ret_d(dut.exe_o.st_data),
	.bw_v(dut.g_bus.u_bcu.done && dut.g_bus.u_bcu.kind == 2'd0 && !m_flt),
	.bw_a(mem_addr), .bw_s(mem_size), .bw_d(mem_wdata),
	.bf_v(dut.g_bus.u_bcu.done && dut.g_bus.u_bcu.kind == 2'd0 && m_flt && dut.g_bus.u_bcu.k_fifo &&
	      dut.g_bus.u_bcu.sp_fin),
	.bf_a(dut.g_bus.u_bcu.sp_on ? dut.g_bus.u_bcu.sp_a : mem_addr),
	.bf_s(dut.g_bus.u_bcu.sp_on ? dut.g_bus.u_bcu.sp_s : mem_size),
	.sf_v(dut.g_bus.u_bcu.done && dut.g_bus.u_bcu.kind == 2'd0 && m_flt && !dut.g_bus.u_bcu.k_fifo &&
	      dut.g_bus.u_bcu.sp_on),
	.sf_n(dut.g_bus.u_bcu.sp_w ? {dut.g_bus.u_bcu.sp_i[0], 1'b0} : dut.g_bus.u_bcu.sp_i),   // (bytes written)
	.br_v(dut.g_bus.u_bcu.go_rd), .br_a(dut.g_bus.u_bcu.dq_a),
	.fr_v(dut.g_bus.u_bcu.rd_fast && dut.g_bus.u_bcu.dq_v), .fr_a(dut.g_bus.u_bcu.dq_a), .fr_s(dut.g_bus.u_bcu.dq_s),
	.occ_n(dut.g_bus.u_bcu.sb_cnt),
	.full_hold(dut.exe_valid && dut.exe_o.st_v && dut.wb_post && dut.sb_full),
	.rd_wait(dut.g_bus.u_bcu.dq_v && dut.g_bus.u_bcu.sb_cnt != 0),
	.f_wait(dut.g_bus.b_f_req && dut.g_bus.u_bcu.sb_cnt != 0),
	.sy_wait(dut.g_bus.u_bcu.st_v && dut.g_bus.u_bcu.sb_cnt != 0)
);
`else
assign ev_rd = dut.u_l1.rd_req;   assign ev_rd_addr = dut.u_l1.rd_addr; assign ev_rd_size = dut.u_l1.rd_size;
assign ev_wr = dut.u_l1.wr_req;   assign ev_wr_addr = dut.u_l1.wr_addr; assign ev_wr_size = dut.u_l1.wr_size;
assign ev_wr_data = dut.u_l1.wr_data;
assign ev_rd_fc = dut.d_rd_fc;    assign ev_wr_fc = dut.exe_o.st_fc;
assign ev_wr_rb = dut.exe_o.st_rb;
`endif

function [31:0] reg_by_name(input [63:0] n);   // key as a right-justified ASCII vector
	case (n)
		"d0": reg_by_name = dbg_d0;  "d1": reg_by_name = dbg_d1;
		"d2": reg_by_name = dbg_d2;  "d3": reg_by_name = dbg_d3;
		"d4": reg_by_name = dbg_d4;  "d5": reg_by_name = dbg_d5;
		"d6": reg_by_name = dbg_d6;  "d7": reg_by_name = dbg_d7;
		"a0": reg_by_name = dut.u_regfile.areg[0];  "a1": reg_by_name = dut.u_regfile.areg[1];
		"a2": reg_by_name = dut.u_regfile.areg[2];  "a3": reg_by_name = dut.u_regfile.areg[3];
		"a4": reg_by_name = dut.u_regfile.areg[4];  "a5": reg_by_name = dut.u_regfile.areg[5];
		"a6": reg_by_name = dut.u_regfile.areg[6];
		"usp": reg_by_name = dut.u_regfile.usp;
		"isp": reg_by_name = dut.u_regfile.isp;
		"msp": reg_by_name = dut.u_regfile.msp;
		"sr":  reg_by_name = {16'd0, dbg_sr};
		"ccr": reg_by_name = {27'd0, dbg_ccr};
		"vbr": reg_by_name = dut.vbr;
		default: reg_by_name = 32'hDEAD_BEEF;
	endcase
endfunction

reg [15:0] img [0:L1_WORDS-1];
string progf, expf;
reg [63:0] key;           // (not a string: Icarus crashes comparing string characters)
integer i, fd, rc, ok, max_cycles, cycles, errors, nchk;
reg [31:0] halt_pc, a, v, got;
// "x80": the value's position on the extended-precision number line, in
// units of the last place.  {exponent, mantissa without the explicit
// integer bit} is monotonic and continuous across binades (the step from
// $FFFF.. at exponent e to $8000.. at e+1 is one ulp), denormals included.
reg [15:0] xse, gse;
reg [63:0] xman, gman;
integer    xn;
reg [77:0] xk_e, xk_g, xk_d;
reg        done = 0;

// "halted": the program must end in a double fault (the core halts,
// M68040UM 7.6.3), not at the halt label
reg want_halt = 0;
// "maxclk <n>": the program must halt within n clocks -- a performance
// regression test (a cycle budget the core has been measured to meet).  It is
// a zero-wait budget: checked on the L1 build only; the bus profiles add
// memory latency by design.
integer max_clk = 0;
always @(posedge clk)
	if (nreset && ((dbg_wb_valid && dbg_wb_pc == halt_pc && !want_halt) || (want_halt && dut.dbg_halted))) done = 1;

// "noread <addr>": no data read may touch that byte (CLR and Scc to memory
// are pure writes on the 68040)
reg [31:0] noread [0:15];
integer    n_noread = 0, reads_bad = 0, nr;
reg [31:0] rlen;
always @(posedge clk)
	if (nreset && ev_rd) begin
		rlen = (ev_rd_size == 2'd0) ? 1 : (ev_rd_size == 2'd1) ? 2 : 4;
		for (nr = 0; nr < n_noread; nr = nr + 1)
			if (noread[nr] >= ev_rd_addr && noread[nr] < ev_rd_addr + rlen) begin
				reads_bad = reads_bad + 1;
				$display("FAIL: data read of %h (size %0d) touches noread byte %h", ev_rd_addr, rlen, noread[nr]);
			end
	end

// "rbcount <n>" + "rb <addr>" lines: exactly n locked write-backs (CAS/CAS2
// mismatch, M68040UM p. 7-26), at those addresses in that order, each
// rewriting the bytes memory holds
integer n_rb = 0, want_rb = -1, rb_bad = 0, kb, n_rbl = 0;
reg [31:0] rb_at [0:31];
reg [2:0] rbn;
always @(posedge clk)
	if (nreset && ev_wr && ev_wr_rb) begin
		if (n_rb < n_rbl && rb_at[n_rb] != ev_wr_addr) begin
			rb_bad = rb_bad + 1;
			$display("FAIL: locked write-back #%0d at %h, expected %h", n_rb, ev_wr_addr, rb_at[n_rb]);
		end
		n_rb = n_rb + 1;
		rbn = (ev_wr_size == 2'd0) ? 1 : (ev_wr_size == 2'd1) ? 2 : 4;
		for (kb = 0; kb < rbn; kb = kb + 1)
			if (mb(ev_wr_addr + kb) !== ev_wr_data[8 * (rbn - 1 - kb) +: 8]) begin
				rb_bad = rb_bad + 1;
				$display("FAIL: locked write-back at %h changes byte %0d", ev_wr_addr, kb);
			end
	end

// "readonce <addr>": exactly one data read touches that byte (MOVEM/MOVEP
// must not read an operand twice: I/O side effects)
reg [31:0] ro_at [0:15];
integer    ro_n [0:15];
integer    n_ro = 0, kr;
always @(posedge clk)
	if (nreset && ev_rd)
		for (kr = 0; kr < n_ro; kr = kr + 1)
			if (ro_at[kr] >= ev_rd_addr &&
			    ro_at[kr] < ev_rd_addr + ((ev_rd_size == 2'd0) ? 1 : (ev_rd_size == 2'd1) ? 2 : 4))
				ro_n[kr] = ro_n[kr] + 1;

// "fc <addr> <fc>": every data read and write touching that byte uses that
// function code (MOVES: SFC/DFC; the rest: 5 supervisor / 1 user data)
reg [31:0] fc_at [0:15];
reg  [2:0] fc_want [0:15];
integer    n_fc = 0, fc_bad = 0, fc_hits = 0, kf;
always @(posedge clk)
	if (nreset) for (kf = 0; kf < n_fc; kf = kf + 1) begin
		if (ev_rd && fc_at[kf] >= ev_rd_addr &&
		    fc_at[kf] < ev_rd_addr + ((ev_rd_size == 2'd0) ? 1 : (ev_rd_size == 2'd1) ? 2 : 4)) begin
			fc_hits = fc_hits + 1;
			if (ev_rd_fc !== fc_want[kf]) begin
				fc_bad = fc_bad + 1;
				$display("FAIL: read of %h with FC %0d, expected %0d", ev_rd_addr, ev_rd_fc, fc_want[kf]);
			end
		end
		if (ev_wr && fc_at[kf] >= ev_wr_addr &&
		    fc_at[kf] < ev_wr_addr + ((ev_wr_size == 2'd0) ? 1 : (ev_wr_size == 2'd1) ? 2 : 4)) begin
			fc_hits = fc_hits + 1;
			if (ev_wr_fc !== fc_want[kf]) begin
				fc_bad = fc_bad + 1;
				$display("FAIL: write of %h with FC %0d, expected %0d", ev_wr_addr, ev_wr_fc, fc_want[kf]);
			end
		end
	end

// "syncpc <addr>": when the instruction at <addr> retires, no store may be
// on its way to memory any more (NOP synchronises, M68040UM 7.7) -- bus mode
reg [31:0] sync_at [0:15];
integer    n_sync = 0, sync_bad = 0, sync_hits = 0, ks;
always @(posedge clk)
	if (nreset && dbg_wb_valid)
		for (ks = 0; ks < n_sync; ks = ks + 1)
			if (dbg_wb_pc == sync_at[ks]) begin
				sync_hits = sync_hits + 1;
`ifdef BUS_MODE
				if (dut.sb_busy) begin
					sync_bad = sync_bad + 1;
					$display("FAIL: %h retired with a store still posted", dbg_wb_pc);
				end
`endif
			end

// "inimage": no data access may fall outside the image.  The L1 model
// aliases the high address bits (ea_all relies on it), so a wrong high
// address (a bitfield offset shifted without its sign, say) would otherwise
// go unseen.
reg inimage = 0;
always @(posedge clk)
	if (nreset && inimage && ((ev_rd && (ev_rd_addr >> (L1_AW + 1)) != 0) ||
	               (ev_wr && (ev_wr_addr >> (L1_AW + 1)) != 0))) begin
		reads_bad = reads_bad + 1;
		if (ev_rd && (ev_rd_addr >> (L1_AW + 1)) != 0)
			$display("FAIL: data read outside the image at %h", ev_rd_addr);
		else
			$display("FAIL: data write outside the image at %h", ev_wr_addr);
	end

initial begin
	if (!$value$plusargs("prog=%s", progf) || !$value$plusargs("expect=%s", expf)) begin
		$display("usage: +prog=<image.hex> +expect=<file.exp> [+cycles=N]");
		$finish;
	end
	if (!$value$plusargs("cycles=%d", max_cycles)) max_cycles = 20000;
	for (i = 0; i < L1_WORDS; i = i + 1) img[i] = 16'h4E71;
	$readmemh(progf, img);
`ifdef BUS_MODE
	if (!$value$plusargs("prof=%d", prof)) prof = 0;
	for (i = 0; i < L1_WORDS; i = i + 1) bmem[i] = img[i];
`else
	for (i = 0; i < L1_WORDS; i = i + 1)
		dut.u_l1.mem[(i - (PC_RESET >> 1)) & (L1_WORDS - 1)] = img[i];
`endif
	// the halt line first
	halt_pc = 32'hFFFF_FFFE;
	fd = $fopen(expf, "r");
	if (fd == 0) begin $display("FAIL: cannot open %0s", expf); $finish; end
	ok = 1;
	while (!$feof(fd) && ok) begin
		ok = ($fscanf(fd, "%s", key) == 1);
		if (!ok) ;
		else if (key == "halt") rc = $fscanf(fd, "%h", halt_pc);
		else begin
			if (key == "#") begin : skipline
				integer ch;
				ch = 0;
				while (ch != 10 && !$feof(fd)) ch = $fgetc(fd);
			end else if (key == "m8" || key == "m16" || key == "m32") rc = $fscanf(fd, "%h %h", a, v);
			else if (key == "noread") begin
				rc = $fscanf(fd, "%h", a);
				noread[n_noread] = a; n_noread = n_noread + 1;
			end else if (key == "inimage") inimage = 1;
			else if (key == "readonce") begin rc = $fscanf(fd, "%h", a); ro_at[n_ro] = a; ro_n[n_ro] = 0; n_ro = n_ro + 1; end
			else if (key == "rbcount") rc = $fscanf(fd, "%d", want_rb);
			else if (key == "halted") want_halt = 1;
			else if (key == "maxclk") rc = $fscanf(fd, "%d", max_clk);
			else if (key == "syncpc") begin rc = $fscanf(fd, "%h", a); sync_at[n_sync] = a; n_sync = n_sync + 1; end
			else if (key == "berr") begin
				rc = $fscanf(fd, "%h %s", a, key);
				be_at[n_be] = a; be_kind[n_be] = key[7:0]; be_used[n_be] = 1'b0; n_be = n_be + 1;
			end
			else if (key == "lberr") begin
				rc = $fscanf(fd, "%h", a);
				be_at[n_be] = a; be_kind[n_be] = "l"; be_used[n_be] = 1'b0; n_be = n_be + 1;
			end
			else if (key == "fc") begin rc = $fscanf(fd, "%h %d", a, v); fc_at[n_fc] = a; fc_want[n_fc] = v[2:0]; n_fc = n_fc + 1; end
			else if (key == "rb") begin rc = $fscanf(fd, "%h", a); rb_at[n_rbl] = a; n_rbl = n_rbl + 1; end
			else if (key == "x80") rc = $fscanf(fd, "%h %h %h %d", a, xse, xman, xn);
			else rc = $fscanf(fd, "%h", v);
		end
	end
	$fclose(fd);
	repeat (3) @(posedge clk);
	nreset = 1;
	for (cycles = 0; cycles < max_cycles && !done; cycles = cycles + 1) @(posedge clk);
	repeat (2) @(posedge clk);
`ifdef BUS_MODE
	// the posted stores the program left behind reach memory
	for (i = 0; i < 2000 && (dut.sb_busy || mem_req || m_busy || m_ack); i = i + 1) @(posedge clk);
	repeat (2) @(posedge clk);
`endif
	errors = 0; nchk = 0;
	if (!done) begin
		errors = errors + 1;
		$display("FAIL: halt pc %h never retired in %0d clocks (last retired %h)", halt_pc, cycles, dbg_wb_pc);
	end
	fd = $fopen(expf, "r");
	ok = 1;
	while (!$feof(fd) && ok) begin
		ok = ($fscanf(fd, "%s", key) == 1);
		if (!ok) ;
		else if (key == "halt") rc = $fscanf(fd, "%h", v);
		else if (key == "noread") rc = $fscanf(fd, "%h", v);
		else if (key == "inimage") ;
		else if (key == "readonce") rc = $fscanf(fd, "%h", v);
		else if (key == "rbcount") rc = $fscanf(fd, "%d", v);
		else if (key == "halted") ;
		else if (key == "maxclk") rc = $fscanf(fd, "%d", v);
		else if (key == "syncpc") rc = $fscanf(fd, "%h", v);
		else if (key == "berr") rc = $fscanf(fd, "%h %s", v, key);
		else if (key == "lberr") rc = $fscanf(fd, "%h", v);
		else if (key == "fc") rc = $fscanf(fd, "%h %d", a, v);
		else if (key == "rb") rc = $fscanf(fd, "%h", v);
		// "fpiar <v>": the unit's FPIAR, which no instruction can read until
		// opclass 100/101 is decoded, so the bench reads it hierarchically.
		// Only the -DFPU_REAL build has a unit to read; any other build that
		// meets the key fails rather than skipping it quietly.
		else if (key == "fpiar") begin
			rc = $fscanf(fd, "%h", v);
			nchk = nchk + 1;
`ifdef FPU_REAL
			got = u_fpu.fpiar;
`else
			got = 32'hDEAD_DEAD;
			$display("FAIL: 'fpiar' needs the -DFPU_REAL build");
`endif
			if (got !== v) begin
				errors = errors + 1;
				$display("FAIL: fpiar = %h, expected %h", got, v);
			end
		end
		else if (key == "x80") begin
			rc = $fscanf(fd, "%h %h %h %d", a, xse, xman, xn);
			nchk = nchk + 1;
			gse  = {mb(a), mb(a + 1)};
			gman = {mb(a + 4), mb(a + 5), mb(a + 6), mb(a + 7), mb(a + 8), mb(a + 9), mb(a + 10), mb(a + 11)};
			xk_e = {xse[14:0], xman[62:0]};
			xk_g = {gse[14:0], gman[62:0]};
			xk_d = (xk_g > xk_e) ? xk_g - xk_e : xk_e - xk_g;
			if (gse[15] !== xse[15] || {mb(a + 2), mb(a + 3)} !== 16'd0 ||
			    gman[63] !== (gse[14:0] != 15'd0) || xk_d > xn) begin
				errors = errors + 1;
				$display("FAIL: x80[%h] = %h_%h, expected %h_%h within %0d ulp (off by %0d)",
				         a, gse, gman, xse, xman, xn, xk_d);
			end
		end
		else if (key == "#") begin : skipline2
			integer ch;
			ch = 0;
			while (ch != 10 && !$feof(fd)) ch = $fgetc(fd);
		end else if (key == "m8" || key == "m16" || key == "m32") begin
			rc = $fscanf(fd, "%h %h", a, v);
			if (key == "m8")       got = {24'd0, mb(a)};
			else if (key == "m16") got = {16'd0, mb(a), mb(a + 1)};
			else                   got = {mb(a), mb(a + 1), mb(a + 2), mb(a + 3)};
			nchk = nchk + 1;
			if (got !== v) begin
				errors = errors + 1;
				$display("FAIL: %0s[%h] = %h, expected %h", key, a, got, v);
			end
		end else begin
			rc = $fscanf(fd, "%h", v);
			got = reg_by_name(key);
			nchk = nchk + 1;
			if (got !== v) begin
				errors = errors + 1;
				$display("FAIL: %0s = %h, expected %h", key, got, v);
			end
		end
	end
	$fclose(fd);
	errors = errors + reads_bad + rb_bad + fc_bad + sync_bad;
	if (n_sync > 0 && sync_hits == 0) begin
		errors = errors + 1;
		$display("FAIL: no 'syncpc' instruction retired");
	end
`ifdef BUS_MODE
	errors = errors + gap_bad;
	u_sbc.settle;
	u_sbc.report("prog");
	errors = errors + u_sbc.bad;
`endif
	if (n_fc > 0 && fc_hits == 0) begin
		errors = errors + 1;
		$display("FAIL: no access touched any 'fc' byte");
	end
	for (kr = 0; kr < n_ro; kr = kr + 1)
		if (ro_n[kr] != 1) begin
			errors = errors + 1;
			$display("FAIL: byte %h read %0d times, expected once", ro_at[kr], ro_n[kr]);
		end
	if (want_rb >= 0 && n_rb != want_rb) begin
		errors = errors + 1;
		$display("FAIL: %0d locked write-backs, expected %0d", n_rb, want_rb);
	end
`ifndef BUS_MODE
	if (max_clk != 0 && cycles > max_clk) begin
		$display("FAIL: halted after %0d clocks, over the budget of %0d (maxclk)", cycles, max_clk);
		errors = errors + 1;
	end
`endif
	if (errors == 0) $display("ALL TESTS PASSED (%0d checks, halt after %0d clocks)", nchk, cycles);
	else $display("%0d CHECK(S) FAILED", errors);
	$finish;
end

endmodule
