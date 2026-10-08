// tb_xc7_check.v - companion top for tb_ap040_pipe_compat.v: the data
// cache's valid, round-robin and dirty tables (vb, rrb, db) checked
// white-box against shadow copies kept here as the flip-flop tables they
// are with AP040_XC7 = 0.  With XC7 = 1 the cache holds them in
// distributed RAM (ap040_xc7_ram2w for vb/rrb, a one-write RAM for db);
// this checker is what says the RAM forms hold, on every clock, exactly
// what the flip-flops would -- a program only notices a wrong valid or
// dirty bit when it happens to change a result.  With XC7 = 0 it checks
// the shadow itself (the tables it compares against ARE the flip-flops).
// A mismatch counts as a bench error (the program then cannot pass).
//
//   iverilog ... -DXC7=1 tb_ap040_pipe_compat.v tb_sb_check.v tb_xc7_check.v <sources>
//
// Compared, at every clock edge (the values before the edge):
//   - every read of vb/rrb the cache makes, through its read wires (vb_q,
//     rrb_q at the lookup/write row; vb_inv at port B's row; vb_f, vb_g,
//     vb_g1 at the IFP/DFP paths' rows) against the shadow at that row;
//   - db at every row the cache reads it (port B's, the walker's, the
//     push's, the request's two, the sweep's, the looked-up row), and all
//     64 rows at the first clock after each reset sweep and at the
//     program's end marker.
// Not compared during the reset sweep (C_SWEEP with sweep_all): the RAM
// has no reset and holds the previous run's bits until the sweep has
// written every row -- which is what the comparison after the sweep
// proves.  The bench resets the cache between its three phases, so every
// program checks that warm reset twice.
//
// Also checked on every clock, in either setting: the dirty table's writes
// never meet in a clock (the merge's set bit, the push's clears, C_TAGW's
// bit) -- the rule the one-write RAM form of db relies on.
`timescale 1ns/1ps
module tb_xc7_check;

`define TB tb_ap040_pipe_compat
`define CC `TB.dut.g_cache.cache

localparam C_TAGW  = 4'd5;
localparam C_SWEEP = 4'd7;
localparam C_PUSH  = 4'd10;

reg [3:0] s_vb  [0:127];
reg [1:0] s_rrb [0:127];
reg [3:0] s_db  [0:63];

integer n_clk = 0, n_cmp = 0, n_bad = 0, n_multi = 0, n_sweeps = 0, n_coll = 0, n_both = 0;
integer k;
reg     armed = 1'b0;       // compared at the edge before
wire    end_mark = `TB.dut.mem_ack && `TB.dut.mem_write && `TB.dut.mem_addr[15:0] == 16'hF102;

wire       ce      = `TB.dut.ce_core;
wire       nreset  = `TB.nreset;
wire       swp_all = (`CC.cst == C_SWEEP) && `CC.sweep_all;
// the dirty table's write statements (ap040_cache.v g_dbff)
wire       w_set   = `CC.st_mrg && `CC.r_cb;
wire       w_err   = (`CC.cst == C_PUSH) && `CC.push_out && `CC.m_err;
wire       w_ack   = (`CC.cst == C_PUSH) && `CC.push_out && `CC.m_ack && (`CC.p_wb == 2'd3);
wire       w_tag   = (`CC.cst == C_TAGW) && !`CC.r_row[6];
wire [3:0] keep    = ((`COPYBACK != 0) && !`CC.inv_idx[6]) ? s_db[`CC.inv_idx[5:0]] : 4'd0;

task bad;
	input [8*12-1:0] what;
	input [6:0]      row;
	input [3:0]      got;
	input [3:0]      exp;
	begin
		n_bad = n_bad + 1;
		`TB.errors = `TB.errors + 1;
		if (n_bad <= 10)
			$display("FAIL: xc7-check: %0s row %0d reads %b, the flip-flop table holds %b (clock %0d)",
			         what, row, got, exp, n_clk);
	end
endtask

// one comparison (a macro, not a task: this runs ~13 times a clock)
`define XC7_CMP(what, row, got, exp) \
	begin n_cmp = n_cmp + 1; if ((got) !== (exp)) bad(what, row, got, exp); end

task db_all;
	begin
		for (k = 0; k < 64; k = k + 1) `XC7_CMP("db", k[6:0], `CC.db[k], s_db[k])
	end
endtask


always @(posedge `TB.clk) begin
	n_clk = n_clk + 1;
	// compare: the cache's reads before this edge against the shadow
	if (nreset && !swp_all) begin
		`XC7_CMP("vb_q",   `CC.vb_ra_q,  `CC.vb_q,   s_vb[`CC.vb_ra_q])
		`XC7_CMP("rrb_q",  `CC.vb_ra_q,  {2'b00, `CC.rrb_q}, {2'b00, s_rrb[`CC.vb_ra_q]})
		`XC7_CMP("vb_inv", `CC.inv_idx,  `CC.vb_inv, s_vb[`CC.inv_idx])
		`XC7_CMP("vb_f",   `CC.vb_ra_f,  `CC.vb_f,   s_vb[`CC.vb_ra_f])
		`XC7_CMP("vb_g",   `CC.vb_ra_g,  `CC.vb_g,   s_vb[`CC.vb_ra_g])
		`XC7_CMP("vb_g1",  `CC.vb_ra_g1, `CC.vb_g1,  s_vb[`CC.vb_ra_g1])
		`XC7_CMP("db_inv",  `CC.inv_idx[5:0],   `CC.db[`CC.inv_idx[5:0]],   s_db[`CC.inv_idx[5:0]])
		`XC7_CMP("db_wk",   `CC.wk_set,         `CC.db[`CC.wk_set],         s_db[`CC.wk_set])
		`XC7_CMP("db_p",    `CC.p_set,          `CC.db[`CC.p_set],          s_db[`CC.p_set])
		`XC7_CMP("db_c1",   `CC.c_addr[9:4],    `CC.db[`CC.c_addr[9:4]],    s_db[`CC.c_addr[9:4]])
		`XC7_CMP("db_c2",   `CC.c_set2,         `CC.db[`CC.c_set2],         s_db[`CC.c_set2])
		`XC7_CMP("db_sw",   `CC.sweep_cnt[5:0], `CC.db[`CC.sweep_cnt[5:0]], s_db[`CC.sweep_cnt[5:0]])
		`XC7_CMP("db_r",    `CC.r_row[5:0],     `CC.db[`CC.r_row[5:0]],     s_db[`CC.r_row[5:0]])
		if (!armed || end_mark) db_all;
	end
	armed <= nreset && !swp_all;
	// the one-write rule of the dirty table
	if (nreset && ce && (`COPYBACK != 0) && (w_set + (w_err | w_ack) + w_tag > 1)) begin
		n_multi = n_multi + 1;
		`TB.errors = `TB.errors + 1;
		if (n_multi <= 10)
			$display("FAIL: xc7-check: two dirty-table writes in one clock (set %b push %b tagw %b, clock %0d)",
			         w_set, w_err | w_ack, w_tag, n_clk);
	end
	if (nreset && swp_all && `CC.sweep_cnt == 7'd127 && ce) n_sweeps = n_sweeps + 1;
	// how often vb/rrb's two writes meet (both ports; the same row: the
	// collision rule), so a run shows what it exercised
	if (nreset && ce && `CC.tag_we && `CC.inv_wren) begin
		n_both = n_both + 1;
		if (`CC.tag_widx == `CC.inv_idx) n_coll = n_coll + 1;
	end
	// update the shadow: ap040_cache.v's flip-flop forms (g_vbff, g_dbff)
	if (!nreset) begin
		for (k = 0; k < 128; k = k + 1) begin
			s_vb[k]  <= 4'd0;
			s_rrb[k] <= 2'd0;
		end
		for (k = 0; k < 64; k = k + 1) s_db[k] <= 4'd0;
	end else begin
		if (ce & `CC.tag_we) begin
			s_vb[`CC.tag_widx]  <= `CC.tag_wdat[91:88];
			s_rrb[`CC.tag_widx] <= `CC.tag_wdat[93:92];
		end
		if (`CC.inv_wren) begin
			s_vb[`CC.inv_idx]  <= s_vb[`CC.inv_idx] & keep;
			s_rrb[`CC.inv_idx] <= 2'd0;
		end
		if (ce && (`COPYBACK != 0)) begin
			if (w_set) s_db[`CC.r_row[5:0]][`CC.hit_way] <= 1'b1;
			if (w_err)
				s_db[`CC.p_set] <= s_db[`CC.p_set] & ~`CC.p_mask;
			else if (w_ack)
				s_db[`CC.p_set][`CC.p_way] <= 1'b0;
			if (w_tag) s_db[`CC.r_row[5:0]][`CC.r_way] <= `CC.wa_ok;
		end
	end
end

// the count, so a run shows the check ran (the program's end marker)
always @(posedge `TB.clk)
	if (end_mark)
		$display("XC7-CHECK: %0d table reads compared over %0d clocks after %0d reset sweeps, %0d mismatches, %0d clocks with two dirty-table writes; valid-table writes on both ports in %0d clocks, %0d on one row",
		         n_cmp, n_clk, n_sweeps, n_bad, n_multi, n_both, n_coll);

endmodule
