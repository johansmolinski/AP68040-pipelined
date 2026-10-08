//--------------------------------------------------------------------------//
// ap040_xc7_ram2w.v - a small table with TWO write ports and NR            //
// asynchronous read ports, built from two single-write banks (the XOR      //
// arrangement): port 0 writes bank 0 with d0 ^ bank1[a0], port 1 writes    //
// bank 1 with d1 ^ bank0[a1], and a read is bank0[a] ^ bank1[a].  Each     //
// bank is a plain array with one write and only variable-address           //
// asynchronous reads, so Vivado infers distributed RAM (RAM64M / RAM64X1D  //
// / RAM128X1D, replicated per read port; UG474 "Distributed RAM").         //
//                                                                          //
// Used only when ap040_pipe_tg68k_compat's AP040_XC7 is on (the cache's    //
// valid and round-robin tables); with it off the tables stay flip-flops    //
// and this module is not instantiated.  Plain Verilog-2001, so every tool  //
// that reads the file list parses it.                                      //
//                                                                          //
// Semantics -- those of the flip-flop table it replaces, written as        //
//   if (we0) t[a0] <= d0;  if (we1) t[a1] <= d1;                           //
//   - a read returns the row's value before this clock's writes;           //
//   - both ports writing the same row in one clock: port 1 wins.           //
// tb/tb_ap040_xc7_ram2w.v checks exactly this against that model.          //
//                                                                          //
// No reset: the caller clears the rows it needs cleared (the cache's       //
// reset sweep does).  The initial blocks start a simulation from zero, as  //
// a configured FPGA's distributed RAM does.  A constant-index read of b0   //
// or b1 anywhere would stop the RAM inference (Synth 8-7186): read the     //
// table through ra/rd only.                                                //
//--------------------------------------------------------------------------//

module ap040_xc7_ram2w
#(
	parameter AW = 7,      // address bits (rows = 2**AW)
	parameter DW = 4,      // data bits per row
	parameter NR = 1       // asynchronous read ports
)
(
	input                  clk,
	input                  we0,
	input       [AW-1:0]   a0,
	input       [DW-1:0]   d0,
	input                  we1,
	input       [AW-1:0]   a1,
	input       [DW-1:0]   d1,
	input    [NR*AW-1:0]   ra,     // read port g's address in ra[g*AW +: AW]
	output   [NR*DW-1:0]   rd      // ... its data in rd[g*DW +: DW]
);

(* ram_style = "distributed" *) reg [DW-1:0] b0 [0:(1<<AW)-1];
(* ram_style = "distributed" *) reg [DW-1:0] b1 [0:(1<<AW)-1];

integer i;
initial begin
	for (i = 0; i < (1 << AW); i = i + 1) begin
		b0[i] = {DW{1'b0}};
		b1[i] = {DW{1'b0}};
	end
end

// the same row written by both: only port 1 writes, over bank 0's old
// value, so the row reads d1
wire same = we0 && we1 && (a0 == a1);

always @(posedge clk)
	if (we0 && !same) b0[a0] <= d0 ^ b1[a0];

always @(posedge clk)
	if (we1) b1[a1] <= d1 ^ b0[a1];

genvar g;
generate
	for (g = 0; g < NR; g = g + 1) begin : gr
		wire [AW-1:0] a = ra[g*AW +: AW];
		assign rd[g*DW +: DW] = b0[a] ^ b1[a];
	end
endgenerate

endmodule
