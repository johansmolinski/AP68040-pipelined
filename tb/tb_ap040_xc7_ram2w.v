// tb_ap040_xc7_ram2w.v - unit bench for rtl/compat/ap040_xc7_ram2w.v, the
// two-write-port table (AP040_XC7) that holds the cache's valid and
// round-robin bits on the Artix-7.  Each instance is compared, every clock,
// against the flip-flop table it replaces: two write statements in one
// always block, port 1 last (so port 1 wins a same-row collision), every
// read the row's value before the clock's writes.
//
// Three instances: the cache's two shapes (128 x 4 with five read ports,
// 128 x 2 with one) and a small one (8 x 3, three read ports) so random
// addresses collide on most clocks.  Stimulus: random enables, addresses
// and data, biased towards both ports naming the same row and towards the
// read ports naming the rows being written; long runs of one port only;
// a zero sweep through port 0 (what the cache's reset sweep does).
//
//   iverilog -g2012 -o x.vvp tb_ap040_xc7_ram2w.v ../rtl/compat/ap040_xc7_ram2w.v
//
// Prints "ALL TESTS PASSED" (and the collision count, so a run shows the
// interesting case was exercised).
`timescale 1ns/1ps

module tb_ap040_xc7_ram2w_one
#(
	parameter AW = 7,
	parameter DW = 4,
	parameter NR = 5,
	parameter SEED = 1
)
(
	input            clk,
	input      [1:0] mode,      // 0 random, 1 port 0 only, 2 port 1 only, 3 zero sweep through port 0
	input            check,
	output reg [31:0] errors,
	output reg [31:0] collisions,
	output reg [31:0] writes
);

reg              we0, we1;
reg  [AW-1:0]    a0, a1;
reg  [DW-1:0]    d0, d1;
reg  [NR*AW-1:0] ra;
wire [NR*DW-1:0] rd;

ap040_xc7_ram2w #(.AW(AW), .DW(DW), .NR(NR)) dut (
	.clk(clk),
	.we0(we0), .a0(a0), .d0(d0),
	.we1(we1), .a1(a1), .d1(d1),
	.ra(ra), .rd(rd));

// the model: the flip-flop table, statement order port 0 then port 1
reg [DW-1:0] mdl [0:(1<<AW)-1];
integer i, seed;
reg [AW-1:0] sweep_a;
initial begin
	seed = SEED;
	errors = 0; collisions = 0; writes = 0;
	sweep_a = 0;
	for (i = 0; i < (1 << AW); i = i + 1) mdl[i] = {DW{1'b0}};
	we0 = 0; we1 = 0; a0 = 0; a1 = 0; d0 = 0; d1 = 0; ra = 0;
end
always @(posedge clk) begin
	if (we0) mdl[a0] <= d0;
	if (we1) mdl[a1] <= d1;
end

// compare every read port against the reference (both settle after the edge)
integer g;
always @(negedge clk) if (check) begin
	for (g = 0; g < NR; g = g + 1)
		if (rd[g*DW +: DW] !== mdl[ra[g*AW +: AW]]) begin
			errors = errors + 1;
			if (errors <= 10)
				$display("FAIL: ram2w AW=%0d DW=%0d NR=%0d: read port %0d row %0d = %h, expected %h (t=%0t)",
				         AW, DW, NR, g, ra[g*AW +: AW], rd[g*DW +: DW], mdl[ra[g*AW +: AW]], $time);
		end
end

// new stimulus after each comparison (away from the edge)
integer r;
always @(negedge clk) begin
	#1;
	r = $random(seed);
	case (mode)
		2'd0: begin we0 = r[0] | r[1]; we1 = r[2] | r[3]; end
		2'd1: begin we0 = r[0] | r[1]; we1 = 1'b0; end
		2'd2: begin we0 = 1'b0;        we1 = r[2] | r[3]; end
		2'd3: begin we0 = 1'b1;        we1 = 1'b0; end
	endcase
	a0 = $random(seed);
	// a1: half the time port 0's row
	a1 = r[4] ? a0 : $random(seed);
	d0 = $random(seed);
	d1 = $random(seed);
	if (mode == 2'd3) begin
		a0 = sweep_a; d0 = {DW{1'b0}};
		sweep_a = sweep_a + 1'b1;
	end
	// read addresses: a quarter each port 0's row, port 1's row, random, and
	// the row the clock before wrote
	for (g = 0; g < NR; g = g + 1) begin
		r = $random(seed);
		case (r[1:0])
			2'd0: ra[g*AW +: AW] = a0;
			2'd1: ra[g*AW +: AW] = a1;
			default: ra[g*AW +: AW] = $random(seed);
		endcase
	end
	if (we0 && we1 && a0 == a1) collisions = collisions + 1;
	writes = writes + we0 + we1;
end

endmodule

module tb_ap040_xc7_ram2w;

reg clk = 0;
always #5 clk = ~clk;

reg [1:0] mode = 2'd0;
reg       check = 1'b0;
wire [31:0] e0, e1, e2, c0, c1, c2, w0, w1, w2;

tb_ap040_xc7_ram2w_one #(.AW(7), .DW(4), .NR(5), .SEED(11)) u_vb  (.clk(clk), .mode(mode), .check(check), .errors(e0), .collisions(c0), .writes(w0));
tb_ap040_xc7_ram2w_one #(.AW(7), .DW(2), .NR(1), .SEED(23)) u_rrb (.clk(clk), .mode(mode), .check(check), .errors(e1), .collisions(c1), .writes(w1));
tb_ap040_xc7_ram2w_one #(.AW(3), .DW(3), .NR(3), .SEED(37)) u_sm  (.clk(clk), .mode(mode), .check(check), .errors(e2), .collisions(c2), .writes(w2));

initial begin
	repeat (2) @(posedge clk);
	check = 1'b1;
	mode = 2'd0; repeat (20000) @(posedge clk);
	mode = 2'd1; repeat (2000)  @(posedge clk);
	mode = 2'd2; repeat (2000)  @(posedge clk);
	mode = 2'd0; repeat (5000)  @(posedge clk);
	// the zero sweep, then random again (a table that came up dirty)
	mode = 2'd3; repeat (128)   @(posedge clk);
	mode = 2'd0; repeat (20000) @(posedge clk);
	@(negedge clk); #2;
	$display("RAM2W: %0d/%0d/%0d writes, %0d/%0d/%0d same-row collisions, %0d/%0d/%0d mismatches",
	         w0, w1, w2, c0, c1, c2, e0, e1, e2);
	if (e0 + e1 + e2 == 0 && c0 > 1000 && c2 > 1000) $display("ALL TESTS PASSED");
	else $display("TEST FAILED");
	$finish;
end

endmodule
