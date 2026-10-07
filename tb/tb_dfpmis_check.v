// tb_dfpmis_check.v - companion top for tb_ap040_pipe_compat.v: the
// misaligned data read path's collision rule (findings/loadstore/plan.md
// step 3, AP040_DFP_MIS), checked white-box at every answer the path gives
// for a two-longword read.  A program cannot see this rule break: a read
// that overlaps a chipset DMA write in time may return either value, and
// the stale copy is never used again once the snoop has invalidated the
// line -- so the rule (no write to either longword's set between the first
// lookup's edge and the answer's, both lines still valid at the answer) is
// asserted here, the way the design means it.  A violation counts as a
// bench error (the program then cannot pass).
//
//   iverilog ... tb_ap040_pipe_compat.v tb_sb_check.v tb_dfpmis_check.v <sources>
//
// Reads: the wrapper's dfp request (the first lookup's edge), the core's
// dfp_m2 (the answer clock of a two-lookup read) and d_rd_fast (the answer
// taken), the cache copy's kept set/way of the first longword and the set of
// the second, the cache's write strobes (tag, data, invalidate -- snoops
// are not clock-enabled) and its valid bits.
`timescale 1ns/1ps
module tb_dfpmis_check;

`define TB tb_ap040_pipe_compat
`define CC `TB.dut.g_cache.cache
`define GD `TB.dut.g_cache.cache.g_dfp
`define CO `TB.dut.core.g_bus

integer t = 0;              // this checker's own clock count (nonblocking: the same value all through an edge)
always @(posedge `TB.clk) t <= t + 1;

integer last_w [0:63];      // clkcount of the last write (tag, data or invalidate) to each data set
integer t_launch = -1;      // clkcount of the edge that launched the current lookup
integer n_served = 0, n_viol = 0;
// diagnostics: how the two-longword lookups fared
integer n_two = 0;          // first lookups of a two-longword read (the cache launched a second)
integer n_two_ok = 0;       // ... the wrapper qualified them with the first half hitting (dfp_two)
integer n_hold = 0;         // ... the core held the slot (store order allowed)
integer n_miss2 = 0;        // ... held, then the second clock missed
// (cache WALLOC) allocates started, merged at validation, filled on the master port
integer n_wa = 0, n_wa_ok = 0, n_wa_fill0 = 0;
integer i;
initial for (i = 0; i < 64; i = i + 1) last_w[i] = -1;

wire ce = `TB.dut.ce_core;
// any write reaching a data-bank set at this edge (as the cache's g_dfp sees them)
wire w_tag = ce & `CC.tag_we & !`CC.tag_widx[6];
wire w_inv = `CC.inv_wren & !`CC.inv_idx[6];
wire w_dat = ce & (|`CC.cd_we) & !`CC.cd_widx[8];
wire [5:0] set1 = `GD.g_set1;
wire [5:0] set2 = `GD.g_set;
wire [3:0] way1 = `GD.g_way1;
wire       hit_mis = ce && `CO.d_rd_fast && `CO.g_dfp.dfp_m2;

always @(posedge `TB.clk) begin
	// the answer of a two-lookup read is taken at this edge: check first,
	// against the writes before this edge, then those at this edge
	if (hit_mis) begin : chk
		integer v0;
		v0 = n_viol;
		n_served = n_served + 1;
		if (!`GD.g_m2) begin
			n_viol = n_viol + 1;
			$display("FAIL: dfpmis-check: answer taken outside the cache's second-lookup clock (t=%0d)", t);
		end
		if (((`CC.vb[{1'b0, set1}] & way1) == 4'd0) || way1 == 4'd0) begin
			n_viol = n_viol + 1;
			$display("FAIL: dfpmis-check: first longword's line (set %0d way %b) not valid at the answer (t=%0d)", set1, way1, t);
		end
		if (last_w[set1] >= t_launch || last_w[set2] >= t_launch) begin
			n_viol = n_viol + 1;
			$display("FAIL: dfpmis-check: a write reached set %0d/%0d between the first lookup (t=%0d) and the answer (t=%0d): last %0d/%0d",
			         set1, set2, t_launch, t, last_w[set1], last_w[set2]);
		end
		if ((w_inv && (`CC.inv_idx[5:0] == set1 || `CC.inv_idx[5:0] == set2)) ||
		    (w_tag && (`CC.tag_widx[5:0] == set1 || `CC.tag_widx[5:0] == set2)) ||
		    (w_dat && (`CC.cd_widx[7:2] == set1 || `CC.cd_widx[7:2] == set2))) begin
			n_viol = n_viol + 1;
			$display("FAIL: dfpmis-check: a write reaches set %0d/%0d in the answer's edge (t=%0d)", set1, set2, t);
		end
		if (n_viol != v0) `TB.errors = `TB.errors + 1;
	end
	// bookkeeping (after the check: a write at this edge is "at the answer")
	if (w_tag) last_w[`CC.tag_widx[5:0]] = t;
	if (w_inv) last_w[`CC.inv_idx[5:0]]  = t;
	if (w_dat) last_w[`CC.cd_widx[7:2]]  = t;
	if (ce && `TB.dut.dfp_req) t_launch = t;
	if (ce && `GD.g_two) begin
		n_two = n_two + 1;
		if (`TB.dut.dfp_two) n_two_ok = n_two_ok + 1;
		if (`CO.d_rd_hold) n_hold = n_hold + 1;
	end
	if (ce && `CO.g_dfp.dfp_m2 && !`CO.d_rd_fast) n_miss2 = n_miss2 + 1;
	// the requester never launches a lookup in clock B (one read
	// outstanding): if it ever does (an LDX change admitting a second early
	// read, say), the second lookup is displaced and the path silently
	// falls back -- fail loudly instead
	// the slot read is HELD while the path works on its second lookup: it
	// must not go out on the port in that clock (it would reach the cache's
	// own misaligned path, which serves the same bytes -- invisible in the
	// values, a protocol break all the same)
	if (ce && `CO.d_rd_hold && `CO.u_bcu.go_rd) begin
		n_viol = n_viol + 1;
		`TB.errors = `TB.errors + 1;
		$display("FAIL: dfpmis-check: the slot read issued in a held clock (t=%0d)", t);
	end
	// (cache MIS) the two-piece machinery's invariants: mis_v is 0 whenever
	// the FSM is idle; C_MST2 never forwards a memory acknowledge to the
	// core; the second piece's launch reads LW1's row and word; the second
	// piece's merge writes LW1's word
	if (ce && `CC.cst == 4'd0 && `CC.mis_v) begin
		n_viol = n_viol + 1; `TB.errors = `TB.errors + 1;
		$display("FAIL: dfpmis-check: mis_v set in C_IDLE (t=%0d)", t);
	end
	if (ce && `CC.cst == 4'd13 && (`CC.pass_active || (`CC.c_ack && !`CC.ack_r))) begin
		n_viol = n_viol + 1; `TB.errors = `TB.errors + 1;
		$display("FAIL: dfpmis-check: C_MST2 passes a memory acknowledge to the core (t=%0d)", t);
	end
	if (ce && `CC.mis_launch2 && (`CC.tag_ridx != `CC.mis_row2 || !`CC.cd_rd_en ||
	                              `CC.cd_ridx != {1'b0, `CC.mis_row2[5:0], `CC.mis_word2})) begin
		n_viol = n_viol + 1; `TB.errors = `TB.errors + 1;
		$display("FAIL: dfpmis-check: the second piece's launch reads the wrong row/word (t=%0d)", t);
	end
	if (ce && `CC.st_merge2 && (`CC.cd_widx != {1'b0, `CC.mis_row2[5:0], `CC.mis_word2} || !`CC.mis_ph)) begin
		n_viol = n_viol + 1; `TB.errors = `TB.errors + 1;
		$display("FAIL: dfpmis-check: the second piece merges into the wrong word (t=%0d)", t);
	end
	// (cache WALLOC) wa_v is 0 whenever the FSM is idle.  A data-bank line
	// fill on the master port (C_FILL, the board's path: FILL_CHANNEL = 0)
	// is a data read (m_instr 0), and the allocate's fill carries the
	// store's FC.  The allocate's fill runs while the core presents its
	// NEXT request -- often an instruction fetch -- so c_instr/c_fc are
	// not the fill's.
	if (ce && `CC.cst == 4'd0 && `CC.wa_v) begin
		n_viol = n_viol + 1; `TB.errors = `TB.errors + 1;
		$display("FAIL: dfpmis-check: wa_v set in C_IDLE (t=%0d)", t);
	end
	if (ce && `CC.cst == 4'd4 && !`CC.r_bank && `CC.m_instr) begin
		n_viol = n_viol + 1; `TB.errors = `TB.errors + 1;
		$display("FAIL: dfpmis-check: a data-bank line fill goes out as an instruction read (t=%0d)", t);
	end
	if (ce && `CC.cst == 4'd4 && `CC.wa_v && `CC.m_fc != `CC.r_fc) begin
		n_viol = n_viol + 1; `TB.errors = `TB.errors + 1;
		$display("FAIL: dfpmis-check: the allocate's fill goes out with FC %0d, the store's is %0d (t=%0d)", `CC.m_fc, `CC.r_fc, t);
	end
	if (ce && `CC.wa_go) n_wa = n_wa + 1;
	if (ce && `CC.wa_ok) n_wa_ok = n_wa_ok + 1;
	if (ce && `CC.wa_v && `CC.cst == 4'd4 && `CC.r_issued && `CC.m_ack && `CC.r_beat == 2'd3) n_wa_fill0 = n_wa_fill0 + 1;
	if (ce && `GD.g_two && `TB.dut.dfp_req) begin
		n_viol = n_viol + 1;
		`TB.errors = `TB.errors + 1;
		$display("FAIL: dfpmis-check: a lookup launched in the second-lookup clock of a two-longword read (t=%0d)", t);
	end
end

// the count, so a run shows the path was exercised
always @(posedge `TB.clk)
	if (`TB.dut.mem_ack && `TB.dut.mem_write && `TB.dut.mem_addr[15:0] == 16'hF102)
		$display("DFPMIS-CHECK: %0d two-longword reads served by the data read path, %0d violations (%0d two-longword lookups: %0d first half hit, %0d held, %0d missed in the second clock)",
		         n_served, n_viol, n_two, n_two_ok, n_hold, n_miss2);
always @(posedge `TB.clk)
	if (`TB.dut.mem_ack && `TB.dut.mem_write && `TB.dut.mem_addr[15:0] == 16'hF102 && n_wa != 0)
		$display("WALLOC-CHECK: %0d store misses allocated, %0d merged at validation (%0d drained), %0d filled on the master port",
		         n_wa, n_wa_ok, n_wa - n_wa_ok, n_wa_fill0);

endmodule
