//--------------------------------------------------------------------------//
// AP040 - MC68040 compatible CPU                                           //
//                                                                          //
// ap040_cache.v - internal instruction and data caches (milestone G)       //
//                                                                          //
// 4KB per side: 64 sets x 4 ways x 16 byte lines, physically tagged        //
// (sits between the MMU and the 16-bit bus adapter). Write-through, no    //
// write-allocate: every store goes to memory, so no dirty state ever      //
// exists and CPUSH degenerates to CINV -- unless COPYBACK is set: then a  //
// store hit on a copyback page stays in the line (dirty) until the line  //
// is pushed (findings/copyback/plan.md, see C_PUSH).  A store that fits one aligned //
// longword and hits a resident data line updates that line in place, as  //
// the 68040 does in write-through mode (plan X3.2); a store that cannot   //
// be merged (misaligned, line-crossing, cache-inhibited, or with the data //
// cache disabled) clears the row it touches instead.  Cacheable reads     //
// must fit inside one aligned longword; misaligned and line-crossing      //
// accesses, walker cycles and cache-inhibited pages bypass the cache.     //
//                                                                          //
// The instruction cache is not snooped by CPU writes (as on the real       //
// 68040): self-modifying code must execute CINV, which invalidates the     //
// whole selected cache (over-invalidation is architecturally safe).        //
//                                                                          //
// Storage: line data in one synchronous RAM (2 banks x 1024 longwords),    //
// tags in one wide synchronous RAM row per {bank, set} (4 ways of 22       //
// bits), valid bits in flip-flops for single-cycle invalidation.           //
//--------------------------------------------------------------------------//

`include "ap040_defs.svh"

module ap040_cache
#(
	// A store to a cacheable page is acknowledged to the core one cycle
	// after acceptance and drains to memory from a latched copy (plan
	// X3.3, A2b-1).  0 keeps every store synchronous, which is the A/B
	// reference: with it the logs must be bit-identical to the tree
	// before the buffer.
	parameter POST_STORES = 1,
	// A miss whose line the wrapper can serve over the line-fill
	// channel (fill_ok) takes the whole line as one payload instead of
	// four longword transactions through the adapter (plan X3.4, A1).
	// 0 keeps every fill on the adapter path: the A/B reference.
	parameter FILL_CHANNEL = 1,
	// Plan M14 (AP68040-pipelined): a second, pipelined READ port into the
	// instruction bank -- see "instruction read path" below.  0 leaves the
	// module exactly as lifted.
	parameter IFP = 0,
	// findings/copyback/plan.md S2: a store that hits a line of a copyback
	// page (c_cb) updates the line and marks it dirty instead of writing
	// memory; a dirty line is written back ("pushed") before it is evicted,
	// before any access that bypasses the cache touches its set, and by
	// every CINV/CPUSH of the data cache.  Needs POST_STORES = 1.  0: the
	// cache is write-through, exactly as before.
	parameter COPYBACK = 0,
	// findings/loadstore/plan.md step 3 (IFP = 1): the data read path also
	// serves a MISALIGNED word or longword that stays inside one 4K page,
	// as two consecutive longword lookups of the D-bank copy (the second in
	// the clock after the first, dfp_two) assembled into the operand in the
	// clock after that.  0: a misaligned read is never a hit, as before.
	parameter DFP_MIS = 0,
	// (MIS) a misaligned data access that stays inside one 4K page is
	// CACHEABLE, as on the 68040 (M68040UM 4.x: a misaligned operand is one
	// cache access per half-line it touches, filled on a miss, merged on a
	// write hit): a read is served (or filled, then served) as two aligned
	// longword pieces -- the two consecutive longwords it spans -- and a
	// store merges its bytes into every resident longword it touches (two
	// pieces, two sets when it crosses the line) instead of clearing the
	// rows.  The memory transfer of a store is the same single misaligned
	// transfer as before; a read miss fills the line(s) as an aligned miss
	// does.  Page-crossing accesses (the next longword in the next page),
	// instruction fetches and everything cache-inhibited keep bypassing
	// exactly as before.  0: misaligned accesses bypass / clear rows.
	parameter MIS = 0,
	// (WALLOC) write-allocate on a copyback store miss (M68040UM 4.4.2, V3/I3):
	// a posted store to a copyback page (r_cb) that misses the data cache
	// fills its line (the dirty victim pushed first, as for a read miss) and
	// merges into it in the clock the line is validated (C_TAGW), leaving the
	// line dirty; the store never reaches memory.  If the line is not
	// validated (a snoop reached its row during the fill or in that clock,
	// ad7008f's rule) the store drains to memory as a write-through instead.
	// Only one-piece stores (not MIS two-piece); a fill error is the late
	// error the core cannot restart (post_err), as a drain's is.  Needs
	// COPYBACK = 1.  0: a copyback store miss writes through, as before.
	parameter WALLOC = 0
)
(
	input             clk,
	input             nreset,
	input             ce,

	input             ie,          // CACR instruction cache enable
	input             de,          // CACR data cache enable

	input             cinv_req,
	input             cinv_ic,
	input             cinv_dc,
	output reg        cinv_done,

	// slave side (from the MMU)
	input             c_req,
	input             c_write,
	input             c_instr,
	input       [1:0] c_size,
	input      [31:0] c_addr,
	input      [31:0] c_wdata,
	input       [2:0] c_fc,
	input             c_nocache,
	input             c_cb,        // (COPYBACK) a copyback page in the copyback window
	// (COPYBACK) the MMU's table walk is in progress.  The walker and this
	// cache share the master port (rtl/soc/TG68K.vhd's mux: wk_go takes it
	// over and takes the next acknowledge as its descriptor).  A write-back
	// starts only with no walk in progress; a pending write-back holds new
	// walks off (post_busy, the MMU's walk_hold).  Found on the board:
	// copyback hung AmigaOS at program load and exit (CacheClearU),
	// 2026-10-01.  A CINV/CPUSH sweep and a walk never overlap at all: the
	// sweep starts with no walk running, and sweep_busy holds walks off
	// while it runs (the instruction fetch past a CPUSH can miss the ATC).
	input             walk_busy,
	// (COPYBACK) the walker's next descriptor transaction is waiting
	// (wk_pend, physical address wk_addr) and may not go out while its set
	// holds a dirty way (wk_wait): M68040UM 3.2.5, a table search sees the
	// data cache's copy and its read-modify-write pushes the line first.
	// From C_IDLE (before a pending CINV/CPUSH, whose sweep waits for the
	// walk) the cache looks the line up (C_WLOOK) and pushes it if it is
	// dirty here -- only that line -- then lets the walker go.
	// The walker has no transaction on the master port while it waits.
	input             wk_pend,
	input      [31:0] wk_addr,
	output            wk_wait,
	output            sweep_busy,   // (COPYBACK) a CINV/CPUSH sweep: no walk may start
	output            c_ack,
	output     [31:0] c_rdata,

	// master side (to the bus adapter)
	output            m_req,
	output            m_write,
	output            m_instr,
	output      [1:0] m_size,
	output     [31:0] m_addr,
	output     [31:0] m_wdata,
	output      [2:0] m_fc,
	input             m_ack,
	input      [31:0] m_rdata,
	// A physical bus error on the transfer this cache issued.  The core
	// samples the same signal and builds its format-$7 frame; the cache
	// must abandon the transfer rather than re-issue it forever.
	input             m_err,

	// instruction read path (IFP = 1): ifp_en at an enabled edge launches a
	// lookup of the set and longword ifp_addr[9:2] names -- inside the page
	// offset, so the LOGICAL address will do (virtually indexed, physically
	// tagged, as the 68040's caches are); the clock after it ifp_thit says
	// whether the instruction bank holds the line whose physical tag is
	// ifp_ptag, and ifp_rdata is that longword.  The requester supplies the
	// tag (its translation) and qualifies the answer (CACR.IE, cacheability).
	input             ifp_en,
	input      [31:0] ifp_addr,
	input      [21:0] ifp_ptag,    // (answer clock) the PHYSICAL tag, [31:10]
	output            ifp_thit,
	output     [31:0] ifp_rdata,

	// data read path (IFP = 1, plan M14 step 3): the same for the DATA
	// bank.  dfp_en launches a lookup of dfp_addr[9:2] (logical: inside the
	// page offset); the clock after it dfp_thit says the data bank holds the
	// line with physical tag dfp_ptag and no write touched that set in the
	// launch edge or touches it now (a fill, a store's merge or invalidate,
	// a snoop), and dfp_rdata is the operand, extracted by dfp_size and the
	// address's low bits as a completed read is.
	input             dfp_en,
	input      [31:0] dfp_addr,
	input       [1:0] dfp_size,
	input      [21:0] dfp_ptag,
	output            dfp_thit,
	output     [31:0] dfp_rdata,
	// (DFP_MIS) in the answer clock: the read spans two longwords and the
	// second is being looked up at this edge; dfp_thit is then the FIRST
	// longword's verdict, and the whole read's answer (dfp_thit, dfp_rdata)
	// comes in the clock after, unless a new lookup is launched meanwhile.
	output            dfp_two,

	// Posted store (A2b-1).  post_busy: a store already acknowledged to
	// the core is still draining to memory; the core's serializing
	// instructions (MOVEC to the MMU, PTEST, PFLUSH) wait for it, and
	// every other access waits by construction because the cache is not
	// idle.  post_err: that drain took a bus error after the core moved
	// on.  The restart model cannot report it -- there is no instruction
	// to restart -- so the core halts; on the configured RAM the buffer
	// posts to, the only bus error is the dead-fabric watchdog anyway.
	output            post_busy,
	output reg        post_err,

	// Line-fill channel (A1).  When fill_ok says the line's physical
	// address is served by a controller with a fill port, a miss raises
	// fill_req with the line address and takes the whole line as one
	// 128-bit payload -- longword at line offset 0 in [127:96], offset
	// 12 in [31:0] -- under fill_ack, a LEVEL held until fill_req drops
	// (the walker bridge's discipline: one toggle each way, payload
	// stable until observed).  fill_err, held the same way, abandons the
	// fill as m_err abandons an adapter fill.  fill_ok = 0 (the wrapper's
	// value until A1-2 wires the channel) keeps every fill on the adapter
	// path, bit for bit as before.
	input             fill_ok,
	output reg        fill_req,
	output     [31:4] fill_addr,
	input     [127:0] fill_data,
	input             fill_ack,
	input             fill_err,

	// Snoop: an external master (chipset DMA, or the MMU table walker)
	// wrote memory behind the CPU's back.  s_stb is a single CLOCK
	// pulse in THIS clock domain, ce-independent, with s_addr held
	// alongside it; the matching data-cache set is invalidated on that
	// clock.  (A ce-gated snoop port was the 5.1 loss: chipset writes
	// landing while clkena is frozen simply vanished.)
	input             s_stb,
	input      [31:0] s_addr
);

//---------------------------------------------------------------------------
// storage
//---------------------------------------------------------------------------

// The tag row holds everything the lookup needs: the four way tags, their
// valid bits and the round-robin victim pointer.  Keeping validity and LRU
// here rather than in flop arrays puts them in M10K with the tags instead
// of in LABs.  The row is carried by the project's true-dual-port dpram
// (rtl/bram.vhd -> altsyncram), so port B can invalidate on a store while
// port A serves lookups and fills; inferring a second write port from a
// bare array does NOT map to M10K and costs ~3000 ALMs instead.
//
//   row = { rr[1:0], valid[3:0], tag3, tag2, tag1, tag0 }   (94 bits)
//
// (findings/copyback/plan.md S1) The lookups no longer read the row's valid
// field: the valid bits are in flip-flops (vb, below), so that a row clear
// can spare the ways that must survive it.
localparam TAGW = 22;
localparam ROWW = 2 + 4 + 4*TAGW;

// One data RAM per way, each {bank, set, word}: reading all four at once
// lets the hit be served in the same cycle the tag compare resolves, so a
// hit costs two cycles instead of three.  Same total bits as the single
// {bank, set, way, word} array it replaces.
(* ramstyle = "no_rw_check" *) reg [31:0] cdata0 [0:511];
(* ramstyle = "no_rw_check" *) reg [31:0] cdata1 [0:511];
(* ramstyle = "no_rw_check" *) reg [31:0] cdata2 [0:511];
(* ramstyle = "no_rw_check" *) reg [31:0] cdata3 [0:511];

wire [ROWW-1:0] tag_q;
reg  [31:0] data_q0, data_q1, data_q2, data_q3;

// RAM control (driven combinationally from the FSM state so the arrays
// infer as block RAM: no resets, enable-gated synchronous reads)
wire        tag_we;
wire  [6:0] tag_ridx, tag_widx;
wire [ROWW-1:0] tag_wdat;
wire        inv_we;              // port B: store invalidation
wire        inv_wren;            // port B write strobe (snoops free-run)
wire  [6:0] inv_idx;
wire        cd_rd_en;
wire  [8:0] cd_ridx, cd_widx;
wire  [3:0] cd_we;               // one per way
wire [31:0] cd_wdat;

// Reads free-run: the address is held for the whole request, so a stalled
// ce simply re-reads the same row.  Only the writes are ce-gated.
dpram #(7, ROWW) ctag_ram
(
	.clock     (clk),
	.address_a (tag_we ? tag_widx : tag_ridx),
	.data_a    (tag_wdat),
	.wren_a    (ce & tag_we),
	.q_a       (tag_q),
	// (findings/copyback/plan.md S1b) port B no longer writes: a row
	// clear is a clear of the valid bits in flops (vb), and the tags stay,
	// so a dirty line's tag survives a clear that spares it
	.address_b (inv_idx),
	.data_b    ({ROWW{1'b0}}),
	.wren_b    (1'b0),
	.q_b       ()
);

always @(posedge clk) begin
	if (ce & cd_we[0]) cdata0[cd_widx] <= cd_wdat;
	if (ce & cd_we[1]) cdata1[cd_widx] <= cd_wdat;
	if (ce & cd_we[2]) cdata2[cd_widx] <= cd_wdat;
	if (ce & cd_we[3]) cdata3[cd_widx] <= cd_wdat;
	if (ce & cd_rd_en) begin
		data_q0 <= cdata0[cd_ridx];
		data_q1 <= cdata1[cd_ridx];
		data_q2 <= cdata2[cd_ridx];
		data_q3 <= cdata3[cd_ridx];
	end
end

//---------------------------------------------------------------------------
// request classification
//---------------------------------------------------------------------------

wire        ena       = c_instr ? ie : de;
// the access must sit inside one aligned longword to be served
wire        fits_long = (c_size == `AP040_SZ_B) ||
                        (c_size == `AP040_SZ_W && !c_addr[0]) ||
                        (c_size == `AP040_SZ_L && c_addr[1:0] == 2'b00);
// (MIS) the window {LW0, LW1} a misaligned access lies in: LW0 is the
// longword c_addr names, LW1 the next one.  mis_two: LW1 is touched (a
// longword at offset 1..3, a word at offset 3); a word at offset 1 lies
// inside LW0 and is a one-piece access (mis_one).  pgx: LW1 is in the next
// 4K page -- a different translation / cache mode: such an access is NOT
// cacheable here (with translation on the BCU has already split it into
// bytes).  Instruction fetches are word-sized and word-aligned or aligned
// longwords; they never take this path.
wire        mis_two_c = (c_size == `AP040_SZ_L && c_addr[1:0] != 2'b00) ||
                        (c_size == `AP040_SZ_W && c_addr[1:0] == 2'b11);
wire        mis_one_c = (c_size == `AP040_SZ_W && c_addr[1:0] == 2'b01);
wire        pgx       = (c_addr[11:2] == 10'h3FF);
wire        mis_acc   = (MIS != 0) && !c_instr && (mis_one_c || (mis_two_c && !pgx));
wire [31:0] mis_addr2_c = {c_addr[31:2], 2'b00} + 32'd4;
wire        bypass    = c_nocache || !ena || c_write || (!fits_long && !mis_acc);
// A store that fits one aligned longword, with the data cache enabled and
// the page cacheable, UPDATES a resident line instead of clearing its row
// (plan X3.2).  Everything else keeps the row invalidate: misaligned and
// line-crossing stores (the second-row machinery exists for them),
// cache-inhibited stores (they may not touch the cache), and stores with
// DE clear (a disabled cache must not be left holding a line memory has
// moved past).  The instruction bank is never touched by a store: the
// 68040 does not snoop its I-cache on CPU writes, CINV covers it.
wire        st_upd_ok = c_write && fits_long && de && !c_nocache && !c_instr;
// (MIS) a misaligned store on the update path: merges into LW0 and LW1
wire        st_mis_ok = c_write && mis_acc && de && !c_nocache && !c_instr;
wire        st_any_upd = st_upd_ok | st_mis_ok;

// Number of bytes following the first byte.  Use a five-bit sum so a
// transfer ending beyond offset 15 cannot wrap before the comparison.
wire  [2:0] write_tail = (c_size == `AP040_SZ_B) ? 3'd0 :
                          (c_size == `AP040_SZ_W) ? 3'd1 : 3'd3;
wire        write_cross_line = ({1'b0, c_addr[3:0]} +
                                 {2'd0, write_tail}) > 5'd15;

wire  [5:0] a_set  = c_addr[9:4];
wire [21:0] a_tag  = c_addr[31:10];
wire  [6:0] a_row  = {c_instr, a_set};

// cacheable read acceptance out of idle (shared with the tag RAM read)
wire rd_accept;

//---------------------------------------------------------------------------
// FSM
//---------------------------------------------------------------------------

localparam C_IDLE  = 4'd0;
localparam C_LOOK  = 4'd1;
localparam C_FERR  = 4'd2;   // aborted fill: invalidate the corrupted row
localparam C_WINV  = 4'd3;   // second-line invalidate owed by a store
localparam C_FILL  = 4'd4;
localparam C_TAGW  = 4'd5;
localparam C_PASS  = 4'd6;
localparam C_SWEEP = 4'd7;   // reset / CINV: walk the rows clearing them
localparam C_FILLC = 4'd8;   // A1: line requested over the fill channel
localparam C_FILLW = 4'd9;   // A1: the delivered line goes into the way
localparam C_PUSH  = 4'd10;  // (COPYBACK) write dirty ways of a data-bank set back
localparam C_WLOOK = 4'd11;  // (COPYBACK) the walker's set: is its line dirty here?
localparam C_MLNCH = 4'd12;  // (MIS) a misaligned read's second longword: its row and word are read
localparam C_MST2  = 4'd13;  // (MIS) a misaligned store's second longword: merge decided
localparam C_WAL   = 4'd14;  // (WALLOC) write-allocate launch: the store's row is read at p_set

// (COPYBACK) Dirty bits, one per data-bank line.  A row clear spares the
// dirty ways (vb_keep): a dirty line is only ever dropped after it has been
// pushed.  Only the DDR3 board's lines can be dirty (the wrapper's window),
// and nothing but the CPU writes that memory, so a snoop never names one.
reg   [3:0] db [0:63];
// The push sequence (C_PUSH): the ways in p_mask of data-bank set p_set,
// lowest first.  For each: four longwords read from the data RAM into
// pbuf (p_ph 0), then written to memory one at a time on the master side
// (p_ph 1), a clock with m_req low between them so each write carries the
// buffer's settled word.  p_ret: where the FSM goes when the mask is empty.
localparam P_IDLE  = 2'd0;   // back to C_IDLE (the held request is re-examined)
localparam P_LOOK  = 2'd1;   // back to C_LOOK (a miss whose victim was dirty)
localparam P_SWEEP = 2'd2;   // back to C_SWEEP (CINV/CPUSH, the same row)
reg   [5:0] p_set;
reg   [3:0] p_mask;
reg   [1:0] p_way;
reg         p_ph;
reg   [2:0] p_rb;
reg   [1:0] p_wb;
reg         p_req;
reg  [21:0] p_tag;
reg  [31:0] pbuf0, pbuf1, pbuf2, pbuf3;
reg   [1:0] p_ret;
function [1:0] low_way;
	input [3:0] m;
	low_way = m[0] ? 2'd0 : m[1] ? 2'd1 : m[2] ? 2'd2 : 2'd3;
endfunction

reg   [3:0] cst;
reg [127:0] fill_line;       // the channel's payload, drained by C_FILLW
// Decoupled drain (plan X3.4, A1-3a).  A posted store leaves C_PASS the
// cycle after its merge and drains to memory from these registers while
// the FSM goes on serving hits; the master side is driven from them for
// as long as dr_active.  Everything that needs memory or the port --
// misses, bypassed accesses, further stores -- waits for the drain, so
// memory order is the program's order and a single slot suffices.
reg         dr_active;
reg  [31:0] dr_addr;
reg  [31:0] dr_wdata;
reg   [1:0] dr_size;
reg   [2:0] dr_fc;
reg   [6:0] sweep_cnt;
reg         sweep_all;   // reset sweep clears both banks
reg         winv_pend;   // a store still owes its second-line invalidate
reg   [5:0] winv_set2;
reg         store_inv_lost;  // a store invalidate that a snoop displaced
reg   [5:0] store_inv_set;
reg   [6:0] r_row;
reg  [21:0] r_tag;
reg   [3:0] r_word;              // {word[1:0]} of the request, plus bank/way
reg   [1:0] r_way;
reg         r_bank;
reg   [1:0] r_beat;
reg         r_issued;
reg  [31:0] r_addr;
reg   [1:0] r_size;
reg   [1:0] r_off;
// (MIS) the two-piece access in progress.  mis_v: the request accepted is
// misaligned (cacheable); mis_two_r: it touches LW1; mis_ph: piece 2 (LW1)
// is the one in progress -- r_row/r_tag/r_word (and r_addr for a read) then
// name LW1's line and word; mis_row1 keeps LW0's set for a store's error
// fallback; mis_lw0 is LW0's data (a read); r_be/r_wd and mis_be2/mis_wd2
// are the store's byte lanes and lane-aligned data for LW0 and LW1.
reg         mis_v, mis_ph, mis_two_r;
reg  [31:0] mis_lw0;
reg   [6:0] mis_row2;
reg  [21:0] mis_tag2;
reg   [1:0] mis_word2;
reg  [31:0] mis_addr2;
reg   [5:0] mis_row1;
reg   [3:0] r_be, mis_be2;
reg  [31:0] r_wd, mis_wd2;
reg         mis_merge1;             // (COPYBACK) LW0's piece merged into a line
reg         st_chk2;                // the clock after st_chk: LW1's row and word have settled
reg         st_snooped2;            // a snoop reached LW1's row since its read was launched
reg  [31:0] fill_hold;           // requested longword captured during fill
reg         ack_r;
reg  [31:0] rdata_r;

// Valid bits (findings/copyback/plan.md S1).  The row's valid field is
// still written, but nothing reads it: the lookup takes the valid bits
// from these flip-flops, which take exactly the row's writes -- port A's
// tag write, and port B's row clear, which wins in the same edge as it does
// in the RAM -- and are read like the row, registered on port A's address
// and old data on a collision (dpram's rdw_mixed OLD_DATA).  In flops, a
// clear can spare ways: vb_keep names the ways a row clear leaves valid --
// the dirty lines (COPYBACK), which are only dropped after a push.
// (S1b) The round-robin victim pointer lives beside them, since port B no
// longer zeroes the row: a row clear resets it, as the zero row did.
reg  [3:0] vb [0:127];
reg  [3:0] vq;
reg  [1:0] rrb [0:127];
reg  [1:0] rq;
wire [3:0] vb_keep = ((COPYBACK != 0) && !inv_idx[6]) ? db[inv_idx[5:0]] : 4'd0;
integer vbk;
always @(posedge clk) begin
	if (!nreset) begin
		for (vbk = 0; vbk < 128; vbk = vbk + 1) begin
			vb[vbk]  <= 4'd0;
			rrb[vbk] <= 2'd0;
		end
	end else begin
		if (ce & tag_we) begin
			vb[tag_widx]  <= tag_wdat[91:88];
			rrb[tag_widx] <= tag_wdat[93:92];
		end
		if (inv_wren) begin
			vb[inv_idx]  <= vb[inv_idx] & vb_keep;
			rrb[inv_idx] <= 2'd0;
		end
	end
	vq <= vb[tag_we ? tag_widx : tag_ridx];
	rq <= rrb[tag_we ? tag_widx : tag_ridx];
end

wire [21:0] t_w0 = tag_q[21:0];
wire [21:0] t_w1 = tag_q[43:22];
wire [21:0] t_w2 = tag_q[65:44];
wire [21:0] t_w3 = tag_q[87:66];
wire v_w0 = vq[0];
wire v_w1 = vq[1];
wire v_w2 = vq[2];
wire v_w3 = vq[3];
wire h0 = v_w0 && (t_w0 == r_tag);
wire h1 = v_w1 && (t_w1 == r_tag);
wire h2 = v_w2 && (t_w2 == r_tag);
wire h3 = v_w3 && (t_w3 == r_tag);
wire      look_hit = h0 | h1 | h2 | h3;
wire [1:0] hit_way = h0 ? 2'd0 : h1 ? 2'd1 : h2 ? 2'd2 : 2'd3;

// store merge into a cached longword: the inverse of lw_extract, the
// store's right-aligned data dropped into its big-endian lane(s)
function [31:0] lw_merge;
	input [31:0] lw;
	input [31:0] wd;
	input [1:0] size;
	input [1:0] off;
	begin
		case (size)
			`AP040_SZ_B:
				case (off)
					2'd0: lw_merge = {wd[7:0], lw[23:0]};
					2'd1: lw_merge = {lw[31:24], wd[7:0], lw[15:0]};
					2'd2: lw_merge = {lw[31:16], wd[7:0], lw[7:0]};
					default: lw_merge = {lw[31:8], wd[7:0]};
				endcase
			`AP040_SZ_W:
				lw_merge = off[1] ? {lw[31:16], wd[15:0]} : {wd[15:0], lw[15:0]};
			default: lw_merge = wd;
		endcase
	end
endfunction

// (MIS) byte-lane merge: be[3] is byte 0 (bits 31:24) .. be[0] byte 3
function [31:0] merge_be;
	input [31:0] lw;
	input [31:0] wd;
	input [3:0] be;
	begin
		merge_be = {be[3] ? wd[31:24] : lw[31:24], be[2] ? wd[23:16] : lw[23:16],
		            be[1] ? wd[15:8]  : lw[15:8],  be[0] ? wd[7:0]   : lw[7:0]};
	end
endfunction
// (MIS) a store's data placed in the 64-bit window {LW0, LW1} at byte
// offset off (M68040UM figure 7-5: the operand is the big-endian bytes at
// A..A+n-1), and the lanes it occupies ([7:4] LW0's, [3:0] LW1's)
function [63:0] win_data;
	input [31:0] wd;
	input [1:0] size;
	input [1:0] off;
	begin
		case (size)
			`AP040_SZ_B: win_data = {wd[7:0], 56'd0} >> {off, 3'd0};
			`AP040_SZ_W: win_data = {wd[15:0], 48'd0} >> {off, 3'd0};
			default:     win_data = {wd, 32'd0} >> {off, 3'd0};
		endcase
	end
endfunction
function [7:0] win_be;
	input [1:0] size;
	input [1:0] off;
	begin
		case (size)
			`AP040_SZ_B: win_be = 8'b1000_0000 >> off;
			`AP040_SZ_W: win_be = 8'b1100_0000 >> off;
			default:     win_be = 8'b1111_0000 >> off;
		endcase
	end
endfunction

// size extraction from a cached longword (big endian lanes)
function [31:0] lw_extract;
	input [31:0] lw;
	input [1:0] size;
	input [1:0] off;
	begin
		case (size)
			`AP040_SZ_B:
				case (off)
					2'd0: lw_extract = {24'd0, lw[31:24]};
					2'd1: lw_extract = {24'd0, lw[23:16]};
					2'd2: lw_extract = {24'd0, lw[15:8]};
					default: lw_extract = {24'd0, lw[7:0]};
				endcase
			`AP040_SZ_W:
				lw_extract = off[1] ? {16'd0, lw[15:0]} : {16'd0, lw[31:16]};
			default: lw_extract = lw;
		endcase
	end
endfunction

// (DFP_MIS) the operand out of a 64-bit window: w0 is the longword the
// address names, w1 the next one.  With two = 0 (one lookup) w1 is unused,
// and the result is lw_extract's for every alignment it serves; a word at
// offset 1 lies inside one longword and is served by one lookup too.
function [31:0] lw_extract2;
	input [31:0] w0;
	input [31:0] w1;
	input [1:0] size;
	input [1:0] off;
	input two;
	reg [31:0] a;
	begin
		a = two ? w0 : w1;   // (two: the first longword was kept in w0; else the lookup's own data arrives in w1)
		case (size)
			`AP040_SZ_B:
				case (off)
					2'd0: lw_extract2 = {24'd0, a[31:24]};
					2'd1: lw_extract2 = {24'd0, a[23:16]};
					2'd2: lw_extract2 = {24'd0, a[15:8]};
					default: lw_extract2 = {24'd0, a[7:0]};
				endcase
			`AP040_SZ_W:
				case (off)
					2'd0: lw_extract2 = {16'd0, a[31:16]};
					2'd1: lw_extract2 = {16'd0, a[23:8]};
					2'd2: lw_extract2 = {16'd0, a[15:0]};
					default: lw_extract2 = {16'd0, a[7:0], w1[31:24]};
				endcase
			default:
				case (off)
					2'd0: lw_extract2 = a;
					2'd1: lw_extract2 = {a[23:0], w1[31:24]};
					2'd2: lw_extract2 = {a[15:0], w1[31:16]};
					default: lw_extract2 = {a[7:0], w1[31:8]};
				endcase
		endcase
	end
endfunction

// Update-path store (X3.2).  st_chk marks the first C_PASS cycle, where
// the tag row read on the store's address has settled and the merge is
// decided; st_flow marks a store that took this path for as long as it
// is in C_PASS, so a bus error on it can drop the row it may have
// updated.  st_snooped is the store's look_snooped: a snoop writing the
// store's row through port B while port A reads it makes the compare
// DONT_CARE on silicon (5.2c), and a false hit would merge the store into
// the WRONG way.  It is armed free-running by any snoop to the row from
// acceptance to the merge cycle and turns the merge into the fallback --
// the row invalidate through the recorded slot (store_inv_lost), which
// already stalls the next store until port B has served it.
// Declared ahead of the collision block below, which reads them: Icarus
// and Quartus bind names in order (hoist_decls.py exists for that).
reg        st_chk;
reg        st_flow;
reg        st_snooped;
wire       wr_accept_upd;
wire       snoop_st_row_acc  = s_stb && (s_addr[9:4] == a_set);
wire       snoop_st_row_pass = s_stb && (s_addr[9:4] == r_row[5:0]);
// Posted store: the store's operands are latched at acceptance (every
// store latches them; the merge reads r_wdata too), st_posted marks a
// C_PASS whose store has already been acknowledged, and the master
// side is driven from the latched copy for as long as it is set.  A
// store is postable when its page is cacheable: that is configured RAM
// behind a snooped path, where the only bus error is the watchdog.
// Cache-inhibited stores (IO, serialized pages) stay synchronous, so an
// IO write still completes before the next instruction.
reg        st_posted;
reg        r_cb;       // (COPYBACK) the update-path store is to a copyback page
reg [31:0] r_wdata;
reg  [2:0] r_fc;
wire       st_post_ok = (POST_STORES != 0) && c_write && !c_nocache &&
                        !c_instr;
wire       sweep_push;   // (COPYBACK) declared here, assigned with need_push below
// (WALLOC) a write-allocate in progress: from the store's miss verdict
// (C_PASS, st_chk) through C_WAL, C_LOOK, the fill and C_TAGW.  The core
// was acknowledged at acceptance; the allocate holds the walker off
// (post_busy) exactly as the drain it replaces would.
reg        wa_v;
assign post_busy = st_posted | dr_active | (cst == C_PUSH) | sweep_push | ((WALLOC != 0) && wa_v);

// Snoop-vs-fill and snoop-vs-lookup collisions (5.2).  A snoop hitting
// the row of an in-flight fill poisons it: the fill's data may predate
// the snooped write, and the tag writeback would compose valid bits
// from a row image the snoop is concurrently changing.  The fill still
// delivers its data to the CPU (read from memory) but the line is not
// validated.  A snoop hitting the row of a lookup in its acceptance or
// compare cycle forces a miss: the row image under the compare is
// mid-change (mixed-port read-during-write is DONT_CARE on silicon),
// and the refill is always safe.  Both flags are set free-running --
// the snoop is -- and consumed/cleared in the ce domain.
wire snoop_fill_row = s_stb && !r_bank && (s_addr[9:4] == r_row[5:0]);
// (MIS) outside C_IDLE the row under the compare is r_row's -- the same
// set as a_set in a lookup's compare clock, LW1's in a misaligned read's
// second piece
wire snoop_look_row = s_stb && !c_instr &&
                      (s_addr[9:4] == (((MIS != 0) && (cst != C_IDLE)) ? r_row[5:0] : a_set));
// (MIS) the second piece's merge clock of a misaligned store, and a snoop
// reaching LW1's row from the launch of its read (the st_chk clock) on
wire mis_m2 = (MIS != 0) && mis_v && mis_two_r && (((cst == C_PASS) && st_chk2) || (cst == C_MST2));
wire snoop_st_row2 = s_stb && (s_addr[9:4] == mis_row2[5:0]);
reg  fill_snooped, look_snooped;
always @(posedge clk) begin
	if (!nreset) begin
		fill_snooped <= 0;
		look_snooped <= 0;
		st_snooped <= 0;
		st_snooped2 <= 0;
	end
	else begin
		if (ce && cst == C_LOOK && (!look_hit || ((WALLOC != 0) && wa_v))) fill_snooped <= 0;
		if ((cst == C_FILL || cst == C_TAGW ||
		     cst == C_FILLC || cst == C_FILLW) && snoop_fill_row)
			fill_snooped <= 1;
		if (ce && (rd_accept || ((MIS != 0) && cst == C_MLNCH))) look_snooped <= 0;
		if ((rd_accept || cst == C_LOOK || ((MIS != 0) && cst == C_MLNCH)) && snoop_look_row)
			look_snooped <= 1;
		// (MIS) LW1's window: from the clock its row is read (st_chk) to its
		// merge; the set wins over the clear
		if (ce && wr_accept_upd) st_snooped2 <= 0;
		if (mis_v && mis_two_r && (((cst == C_PASS) && st_chk) || mis_m2) && snoop_st_row2)
			st_snooped2 <= 1;
		// the store's window runs from its acceptance to the merge
		// cycle; the set wins over the clear, as for look_snooped
		if (ce && wr_accept_upd) st_snooped <= 0;
		if ((wr_accept_upd && snoop_st_row_acc) ||
		    (st_chk && (cst == C_PASS) && snoop_st_row_pass))
			st_snooped <= 1;
	end
end
//---------------------------------------------------------------------------
// forwarding
//---------------------------------------------------------------------------

// A passed access is forwarded from C_PASS ONLY.  Accepting and acking
// one in C_IDLE used to save a cycle, but it made c_ack combinational in
// c_req -- and c_req carries the ATC compare, so the core's mem_ack (and
// with it the whole 47-level exception-format mux it gates) hung off the
// ATC block RAM output in the same cycle.  That single path cost 5.9 ns
// and was the entire reason the internal caches could not be enabled.
// ap040_mmu already refuses the same shortcut for the same reason -- see
// its c_ack comment.  The cost is one cycle per BYPASSED access (I/O,
// misaligned, cache-inhibited); cacheable traffic goes through C_LOOK
// and is untouched.
wire pass_active = (cst == C_PASS);
wire fill_active = (cst == C_FILL);
// the channel's line, one longword per C_FILLW cycle, offset 0 first
wire [31:0] fill_beat = (r_beat == 2'd0) ? fill_line[127:96] :
                        (r_beat == 2'd1) ? fill_line[95:64]  :
                        (r_beat == 2'd2) ? fill_line[63:32]  : fill_line[31:0];
assign fill_addr = r_addr[31:4];

// Set when a transfer this cache issued took a bus error; cleared when
// the core withdraws the faulted request.  Without it the level-held
// request would be re-accepted on the very next cycle and re-issued to
// the address that just faulted.
reg  err_hold;
// A cache-inhibited READ that hits a resident line must invalidate it
// while it bypasses (WinUAE dcache040: a hit under CACHE_DISABLE_MMU is
// pushed and invalidated before the uncached access; the icache path
// invalidates likewise).  Leaving the line valid let stale data hit
// again when the mapping turned cacheable.  Stores need nothing extra:
// every accepted store already clears its row (store_inv).  The
// invalidate is recorded here and served through port B whenever the
// port is free; new cacheable reads are held off until it lands, so the
// stale line cannot be re-hit in the window.
reg        pass_ci_chk;   // first C_PASS cycle of a CI read: tags valid
reg        ci_inv_pend;   // a CI hit awaits its row invalidate
reg  [6:0] ci_inv_row;

// While a posted store drains, the master side is the latched store:
// the core has dropped its request and may already present the next
// one, which the cache does not look at until it is idle again.
// The drain owns the master side outright while it runs: nothing else can
// be in C_FILL or C_PASS then, because misses and bypassed accesses are
// held until it ends.  The handover is seamless -- dr_* holds the same
// address and data the first C_PASS cycle presented, so an adapter that
// already latched the request sees no change.
// (COPYBACK) A push owns the master side while it writes (push_out): it
// starts only with the drain idle, and no fill or pass runs meanwhile.  A
// copyback store is not issued in its first C_PASS clock -- that clock
// decides whether it hit (and stays in the cache) or drains to memory.
wire        push_out = (cst == C_PUSH) && p_ph && p_req;
wire [31:0] pbuf_w   = (p_wb == 2'd0) ? pbuf0 : (p_wb == 2'd1) ? pbuf1 :
                       (p_wb == 2'd2) ? pbuf2 : pbuf3;
assign m_req   = dr_active   ? 1'b1 :
                 push_out    ? 1'b1 :
                 fill_active ? 1'b1 :
                 (pass_active ? ((c_req & ~st_posted) | (st_posted & ~(st_chk & r_cb))) : 1'b0);
assign m_write = dr_active   ? 1'b1 :
                 push_out    ? 1'b1 :
                 fill_active ? 1'b0 : (st_posted ? 1'b1 : c_write);
// (WALLOC) the allocate's fill is a data read with the store's FC: the
// core presents its NEXT request meanwhile (often an instruction fetch),
// so c_instr/c_fc are not the fill's (FILL_CHANNEL = 0, the board's path).
// No field of the fill may follow the core at all: c_wdata moves too when
// that next request is a store, and the SoC latches every m_* field across
// its clock crossing for as long as an access is outstanding (TG68K.vhd
// q_req, sim/ddr3_cpu FC-1/FC-1b).  While wa_v the fill's every field is
// the cache's own: r_wdata/r_fc, the store's, held from wa_go to C_TAGW.
wire        wa_m    = (WALLOC != 0) && wa_v;
assign m_instr = (dr_active | st_posted | push_out | wa_m) ? 1'b0 : c_instr;
assign m_size  = dr_active   ? dr_size :
                 push_out    ? `AP040_SZ_L :
                 fill_active ? `AP040_SZ_L : (st_posted ? r_size : c_size);
assign m_addr  = dr_active   ? dr_addr :
                 push_out    ? {p_tag, p_set, p_wb, 2'b00} :
                 fill_active ? {r_addr[31:4], r_beat, 2'b00}
                             : (st_posted ? r_addr : c_addr);
assign m_wdata = dr_active ? dr_wdata : push_out ? pbuf_w : ((st_posted | wa_m) ? r_wdata : c_wdata);
assign m_fc    = dr_active ? dr_fc    : push_out ? 3'd5   : ((st_posted | wa_m) ? r_fc : c_fc);

// A posted store was acknowledged from ack_r at acceptance; the memory
// acknowledge that ends its drain must NOT reach the core, which by then
// may be holding an unrelated request that would take it as its own.
assign c_ack   = (pass_active && !st_posted) ? m_ack : ack_r;
assign c_rdata = pass_active ? m_rdata : rdata_r;

// (COPYBACK) An access that bypasses the cache -- a read that cannot be
// served, a store that cannot merge -- would read or write memory under a
// dirty line of its set (or of the next set, crossing the line).  Before it
// is accepted, the dirty ways of that set are pushed; the request is still
// held, and C_IDLE looks at it again with the set clean.
wire  [5:0] c_set2   = c_addr[9:4] + 6'd1;
// (COPYBACK) the walker's descriptor in a set with dirty ways: C_WLOOK
// reads the set's tags, and only a dirty way holding the descriptor's own
// line is pushed (the 68040 looks the line up; a dirty neighbour stays).
// wk_chk: this transaction's set has been looked at; it clears when the
// walker's request goes out (wk_pend falls).
reg         wk_chk;
reg         wl_ph;
wire  [5:0] wk_set   = wk_addr[9:4];
wire        wk_push  = (COPYBACK != 0) && wk_pend && (db[wk_set] != 4'd0) && !wk_chk;
assign      wk_wait  = (COPYBACK != 0) && wk_pend &&
                       (((db[wk_set] != 4'd0) && !wk_chk) || (cst == C_WLOOK) || (cst == C_PUSH));
wire  [3:0] wl_match = {db[p_set][3] && (t_w3 == wk_addr[31:10]),
                        db[p_set][2] && (t_w2 == wk_addr[31:10]),
                        db[p_set][1] && (t_w1 == wk_addr[31:10]),
                        db[p_set][0] && (t_w0 == wk_addr[31:10])};
assign      sweep_busy = (COPYBACK != 0) && (cst == C_SWEEP);
wire  [3:0] c_d1     = db[c_addr[9:4]];
wire  [3:0] c_d2     = db[c_set2];
wire        bp_acc   = c_write ? !st_any_upd : (bypass && !c_instr);
wire        need_push = (COPYBACK != 0) && (cst == C_IDLE) && !(cinv_req && !cinv_done) &&
                        c_req && !ack_r && !err_hold && !dr_active && !walk_busy && bp_acc &&
                        ((c_d1 != 4'd0) || (write_cross_line && (c_d2 != 4'd0)));
wire  [5:0] np_set   = (c_d1 != 4'd0) ? c_addr[9:4] : c_set2;
wire  [3:0] np_mask  = (c_d1 != 4'd0) ? c_d1 : c_d2;
// CINV/CPUSH of the data cache: a row with dirty ways is pushed before the
// sweep clears it (findings/copyback/plan.md revision 3: CINV as well)
assign      sweep_push = (COPYBACK != 0) && (cst == C_SWEEP) && !sweep_all &&
                         !sweep_cnt[6] && cinv_dc &&
                         (db[sweep_cnt[5:0]] != 4'd0);

assign rd_accept = (cst == C_IDLE) && !(cinv_req && !cinv_done) &&
                   c_req && !ack_r && !c_write && !bypass && !ci_inv_pend;
// the FSM's acceptance of an update-path store, term for term: it must
// name exactly the cycle the C_IDLE write branch takes that path
assign wr_accept_upd = (cst == C_IDLE) && !(cinv_req && !cinv_done) &&
                       c_req && !ack_r && !err_hold && !store_inv_lost &&
                       !dr_active && st_any_upd;

// a push reads its set's tags (the way's address) through port A
// (MIS) the second piece's row: read in the launch clock (C_MLNCH for a
// read; a store's st_chk clock), and re-read -- instead of the request's
// first row -- for as long as piece 2 is in progress (a fill of LW1's line
// writes its row image back at C_TAGW; a miss waiting for the drain
// re-reads it)
wire        mis_launch2 = (MIS != 0) && ((cst == C_MLNCH) ||
                          ((cst == C_PASS) && st_chk && mis_v && mis_two_r));
// (WALLOC) the allocate reads the store's row at p_set: the core dropped
// the posted store's request, so a_row names whatever it presents now
assign tag_ridx  = ((cst == C_PUSH) || (cst == C_WLOOK) || ((WALLOC != 0) && wa_v)) ? {1'b0, p_set} :
                   (mis_launch2 || ((MIS != 0) && mis_v && mis_ph)) ? mis_row2 : a_row;
wire [87:0] tags_next = (r_way == 2'd0) ? {tag_q[87:22], r_tag} :
                        (r_way == 2'd1) ? {tag_q[87:44], r_tag, tag_q[21:0]} :
                        (r_way == 2'd2) ? {tag_q[87:66], r_tag, tag_q[43:0]} :
                                          {r_tag, tag_q[65:0]};
wire  [3:0] val_next  = vq | (4'd1 << r_way);
wire        sweep_hit = sweep_all || (sweep_cnt[6] ? cinv_ic : cinv_dc);
assign tag_we    = ((cst == C_TAGW) && !fill_snooped && !snoop_fill_row) ||
                   ((cst == C_SWEEP) && sweep_hit && !sweep_push);
assign tag_widx  = (cst == C_SWEEP) ? sweep_cnt : r_row;
assign tag_wdat  = (cst == C_SWEEP) ? {ROWW{1'b0}}
                                    : {rq + 2'd1, val_next, tags_next};

// Port B: a store invalidates the data-bank set it touches, and the next
// set when the transfer crosses the line.  A cleared row needs no
// read-modify-write -- the tags left behind are never consulted without
// their valid bit.  The 68040 leaves the instruction cache alone here.
// Port B invalidates: a snoop takes priority over a store's own
// invalidate, because a missed snoop leaves stale data while a delayed
// store invalidate is picked up again from snoop_pend below.
// An update-path store (st_upd_ok) owes no row invalidate at all: it
// merges into the line on a hit and allocates nothing on a miss.
wire store_inv = ((cst == C_IDLE) && c_req && c_write && !ack_r &&
                  !store_inv_lost && !st_any_upd && !dr_active && !need_push) ||
                 ((cst == C_PASS) && winv_pend) ||
                 (cst == C_WINV);
// Snoop invalidates are FREE-RUNNING (5.1): a chipset write must land
// even while clkena is frozen.  The store-side invalidates stay in the
// ce domain with the FSM that generates them.  The only suppression is
// a sweep zeroing the same row in the same cycle (both write zero; the
// double write is avoided, the effect is identical).
wire snoop_wr  = s_stb && !((cst == C_SWEEP) && sweep_hit &&
                            (sweep_cnt == {1'b0, s_addr[9:4]}));
// An aborted refill has already written its beats into the victim way's
// data RAM while that way still carries its PREVIOUS tag and valid bit.
// Only C_TAGW validates a line, so the incoming line stays unreachable --
// but the line it was evicting does NOT: it keeps hitting on its old tag
// over data the dead fill overwrote.  A user-side miss that bus-errors
// mid-fill therefore hands the next SUPERVISOR hit on that row a mixture
// of kernel tag and user data.  Clear the whole row through port B, which
// writes a constant zero and so cannot race the live tag read the way a
// port A read-modify-write would.  Over-invalidation is correctness-safe.
wire fill_err_inv = (cst == C_FERR) && !snoop_wr;
// lowest priority: the zero-row write is idempotent, so waiting is safe
wire ci_inv = ci_inv_pend && !snoop_wr && !store_inv && !store_inv_lost &&
              !fill_err_inv;

assign inv_we   = snoop_wr || store_inv || store_inv_lost || fill_err_inv ||
                  ci_inv;
assign inv_wren = snoop_wr | (ce & (store_inv | store_inv_lost | fill_err_inv |
                                    ci_inv));
assign inv_idx  = snoop_wr        ? {1'b0, s_addr[9:4]} :
                  fill_err_inv    ? r_row :   // the fill's own bank and row
                  ci_inv          ? ci_inv_row :
                  store_inv_lost ? {1'b0, store_inv_set} :
                  (cst == C_IDLE) ? {1'b0, c_addr[9:4]} : {1'b0, winv_set2};
// the four ways arrive together; the tag compare picks one
wire [31:0] data_hit = (hit_way == 2'd0) ? data_q0 :
                       (hit_way == 2'd1) ? data_q1 :
                       (hit_way == 2'd2) ? data_q2 : data_q3;

// The merge lands in the first C_PASS cycle of an update-path store: the
// tag row and the four ways' words were read with the acceptance (the
// same reads a cacheable lookup issues), so a hit selects its way and
// the store's lanes drop into that word.  A snooped window (st_snooped)
// or a bus error already reported in this cycle suppresses it; the
// memory write itself is untouched either way.  A store that misses
// allocates nothing.
// (COPYBACK) A snoop spares a dirty way and no longer changes a tag, so a
// hit on a dirty way is trusted even in a snooped window: the store must
// merge there, or the dirty line would later overwrite it in memory.
wire hit_dirty = (COPYBACK != 0) && !r_row[6] && db[r_row[5:0]][hit_way];
// A snoop landing IN the merge clock (snoop_st_row_pass) refuses the merge
// too, as C_LOOK's snoop_look_row refuses a hit: st_snooped only records it
// for the clock after.  Without this a copyback store merged into a clean
// way and set its dirty bit in the same edge the snoop cleared the way's
// valid bit (vb_keep reads the dirty bits before the edge): the line went
// invalid-but-dirty, the next read refilled the stale line from memory
// into another way, and the store was lost (t_cbsnp_pipe.s).  Refused, the
// store drains to memory as a write-through and the snoop drops the clean
// line -- what the snoop does to every clean way of its set.  A hit on a
// dirty way is still trusted: a snoop spares it.
wire st_merge = (cst == C_PASS) && st_chk && look_hit &&
                ((!st_snooped && !snoop_st_row_pass) || hit_dirty) && !m_err;
// (MIS) the second piece's merge (LW1, mis_m2), decided on LW1's row image
// against its tag, trusting its own snoop window -- a snoop to LW1's row in
// this very clock too (the snoop would clear the way in the edge the merge
// writes it: a copyback dirty bit on an invalid line).  st_mrg: either.
wire st_merge2 = mis_m2 && look_hit && ((!st_snooped2 && !snoop_st_row2) || hit_dirty) && !m_err;
wire st_mrg    = st_merge | st_merge2;
// (WALLOC) the store's miss verdict, in the clock st_merge is decided: a
// posted copyback one-piece store that missed, with no snoop in its window
// (a snooped window takes the row-invalidate fallback as before)
wire wa_go     = (WALLOC != 0) && (cst == C_PASS) && st_chk && st_posted && r_cb &&
                 !mis_v && !look_hit && !st_snooped && !snoop_st_row_pass && !m_err;
// (WALLOC) the allocate's line is validated in this C_TAGW clock: the same
// condition as tag_we's, so the merge and the valid bit land together
wire wa_ok     = (WALLOC != 0) && (cst == C_TAGW) && wa_v && !fill_snooped && !snoop_fill_row;

// the data-array read runs with the acceptance of a cacheable read AND of
// an update-path store (a store carries c_instr = 0, so its row is the
// data bank's)
wire   push_rd   = (cst == C_PUSH) && !p_ph;
assign cd_rd_en  = rd_accept | wr_accept_upd | push_rd | mis_launch2;
assign cd_ridx   = push_rd ? {1'b0, p_set, p_rb[1:0]} :
                   mis_launch2 ? {1'b0, mis_row2[5:0], mis_word2} : {c_instr, a_set, c_addr[3:2]};
// the pushed way's longword, one clock after its address
wire [31:0] p_dsel = (p_way == 2'd0) ? data_q0 : (p_way == 2'd1) ? data_q1 :
                     (p_way == 2'd2) ? data_q2 : data_q3;
wire [21:0] p_tsel = (p_way == 2'd0) ? t_w0 : (p_way == 2'd1) ? t_w1 :
                     (p_way == 2'd2) ? t_w2 : t_w3;
// C_LOOK trusts a hit on a dirty way in a snooped window, as st_merge does
wire look_dirty = (COPYBACK != 0) && !r_row[6] && db[r_row[5:0]][hit_way];

// dirty bits: set by a copyback store's merge, cleared when the way is
// pushed (or its push fails) and when a fill replaces the line
integer dbk;
always @(posedge clk) begin
	if (!nreset) begin
		for (dbk = 0; dbk < 64; dbk = dbk + 1) db[dbk] <= 4'd0;
	end else if (ce && (COPYBACK != 0)) begin
		if (st_mrg && r_cb) db[r_row[5:0]][hit_way] <= 1'b1;
		if ((cst == C_PUSH) && push_out && m_err)
			db[p_set] <= db[p_set] & ~p_mask;
		else if ((cst == C_PUSH) && push_out && m_ack && (p_wb == 2'd3))
			db[p_set][p_way] <= 1'b0;
		if ((cst == C_TAGW) && !r_row[6]) db[r_row[5:0]][r_way] <= wa_ok;
	end
end
assign cd_we     = ((cst == C_FILL) && r_issued && m_ack) ? (4'd1 << r_way) :
                   (cst == C_FILLW)                        ? (4'd1 << r_way) :
                   st_mrg                                  ? (4'd1 << hit_way) :
                   wa_ok                                   ? (4'd1 << r_way) :
                                                             4'd0;
assign cd_widx   = st_mrg ? {1'b0, r_row[5:0], ((MIS != 0) ? r_word[1:0] : r_addr[3:2])} :
                   wa_ok  ? {1'b0, r_row[5:0], r_addr[3:2]}
                          : {r_bank, r_row[5:0], r_beat};
assign cd_wdat   = st_mrg           ? (((MIS != 0) && mis_v) ? merge_be(data_hit, r_wd, r_be)
                                                             : lw_merge(data_hit, r_wdata, r_size, r_off)) :
                   wa_ok            ? lw_merge(fill_hold, r_wdata, r_size, r_off) :
                   (cst == C_FILLW) ? fill_beat :
                                      m_rdata;



always @(posedge clk) begin
	if (!nreset) begin
		// the tag RAM has no reset, so sweep it clear before serving
		// anything: a garbage row would otherwise read back as a hit
		cst <= C_SWEEP;
		sweep_cnt <= 0;
		sweep_all <= 1;
		winv_pend <= 0;
		winv_set2 <= 0;
		err_hold <= 0;
		pass_ci_chk <= 0;
		ci_inv_pend <= 0;
		ci_inv_row <= 0;
		st_chk <= 0;
		st_flow <= 0;
		st_posted <= 0;
		r_cb <= 0;
		p_set <= 0; p_mask <= 0; p_way <= 0; p_ph <= 0; p_rb <= 0; p_wb <= 0;
		p_req <= 0; p_tag <= 0; p_ret <= P_IDLE;
		wk_chk <= 0; wl_ph <= 0; wa_v <= 0;
		pbuf0 <= 0; pbuf1 <= 0; pbuf2 <= 0; pbuf3 <= 0;
		post_err <= 0;
		r_wdata <= 0;
		r_fc <= 0;
		fill_req <= 0;
		fill_line <= 0;
		dr_active <= 0;
		dr_addr <= 0; dr_wdata <= 0; dr_size <= 0; dr_fc <= 0;
		store_inv_lost <= 0;
		store_inv_set <= 0;
		cinv_done <= 0;
		r_row <= 0; r_tag <= 0; r_word <= 0; r_way <= 0; r_bank <= 0;
		mis_v <= 0; mis_ph <= 0; mis_two_r <= 0; mis_lw0 <= 0; mis_row2 <= 0; mis_tag2 <= 0;
		mis_word2 <= 0; mis_addr2 <= 0; mis_row1 <= 0; r_be <= 0; mis_be2 <= 0; r_wd <= 0; mis_wd2 <= 0;
		mis_merge1 <= 0; st_chk2 <= 0;
		r_beat <= 0; r_issued <= 0; r_addr <= 0; r_size <= 0; r_off <= 0;
		fill_hold <= 0; ack_r <= 0; rdata_r <= 0;
	end
	else if (ce) begin
		ack_r <= 0;
		cinv_done <= 0;
		post_err <= 0;
		st_chk2 <= (MIS != 0) && (cst == C_PASS) && st_chk && mis_v && mis_two_r;
		if (!wk_pend) wk_chk <= 1'b0;
		if (ci_inv) ci_inv_pend <= 0;

		// the decoupled drain completes, or fails, whatever the FSM is
		// doing; a failure is the late error the core cannot restart
		if (dr_active && m_err) begin
			dr_active <= 0;
			post_err  <= 1;
		end
		else if (dr_active && m_ack) dr_active <= 0;

		// A snoop displaced a store's first-set invalidate in its
		// acceptance cycle: remember it and issue it as soon as port B
		// is free.  The second-set (winv) invalidate needs no recording:
		// winv_pend persists until port B actually serves it.  A NEW
		// store is stalled one cycle while a recorded invalidate waits
		// (store_inv's !store_inv_lost term), so the single slot cannot
		// be overwritten.
		if (snoop_wr && (cst == C_IDLE) && c_req && c_write && !ack_r &&
		    !store_inv_lost && !st_any_upd && !dr_active && !need_push) begin
			store_inv_lost <= 1;
			store_inv_set  <= c_addr[9:4];
		end
		else if (store_inv_lost && !s_stb)
			store_inv_lost <= 0;

		case (cst)
			C_IDLE: begin
				if (!c_req) err_hold <= 0;
				if (wk_push && !dr_active) begin
					// (COPYBACK) the walker waits on a set with dirty ways
					p_set <= wk_set;
					wl_ph <= 1'b0;
					cst   <= C_WLOOK;
				end
				// (COPYBACK) a sweep and a table walk never overlap: the
				// sweep starts with no walk running (the walker may be
				// waiting for the push above), and no walk starts while
				// it runs (sweep_busy holds walks off)
				else if (cinv_req && !cinv_done && !((COPYBACK != 0) && walk_busy)) begin
					sweep_cnt <= 0;
					sweep_all <= 0;   // honour the cinv_ic/cinv_dc selects
					cst <= C_SWEEP;
				end
				// (COPYBACK) a bypassing access over a dirty set: push the
				// set's dirty ways first, then look at the request again
				else if (need_push) begin
					p_set  <= np_set;
					p_mask <= np_mask;
					p_way  <= low_way(np_mask);
					p_ph   <= 1'b0;
					p_rb   <= 3'd0;
					p_req  <= 1'b0;
					p_ret  <= P_IDLE;
					cst    <= C_PUSH;
				end
				// A cache-inhibited hit owes a row invalidate.  Accept
				// NOTHING until it lands.  rd_accept alone gated only the
				// data-RAM read enable, so on paper the FSM could still
				// enter C_LOOK and compare against the not-yet-invalidated
				// tag row using stale data_q.
				//
				// WRITES ARE EXEMPT, and must be.  store_inv asserts
				// combinationally while a store waits in C_IDLE and it
				// blocks ci_inv; holding the store as well made the two
				// block each other with no way out -- a hard wedge, the
				// worst possible failure for a cache.  A store needs no
				// exemption from the guarantee anyway: it never reads
				// data_q, and it clears its own row on acceptance.
				// Exempting it also lets ci_inv fire the moment the FSM
				// leaves C_IDLE.
				//
				// HONEST NOTE: that window could not be demonstrated.
				// ci_inv_pend is raised on the FIRST cycle of C_PASS while
				// the access itself runs to m_ack, so the invalidate lands
				// during the memory latency -- before the FSM can accept
				// anything.  A back-to-back request pair with a port-B
				// stealing snoop swept across the completion (T8) passes
				// with and without this guard.  It is kept as
				// defence-in-depth: it makes the module enforce its own
				// contract instead of depending on the caller inserting a
				// request-low cycle, and it keeps ci_inv_row single-slot
				// so a second CI hit cannot overwrite a pending row and
				// lose its invalidate.  The cost is nil in practice.
				else if (c_req && !ack_r && !err_hold &&
				         (c_write || !ci_inv_pend)) begin
					if (c_write) begin
						if (store_inv_lost) begin
							// port B owes a recorded invalidate: hold the
							// store one cycle so its own invalidate cannot
							// be skipped (the request is level-held)
						end
						else if (dr_active) begin
							// the one drain slot is taken: the next store
							// waits, so stores reach memory in order
						end
						else if (st_any_upd) begin
							// update path (X3.2): no row invalidate.  The
							// tag row and the four ways' words are read
							// alongside this acceptance (wr_accept_upd);
							// the first C_PASS cycle decides the merge.
							// (MIS) a misaligned store: LW0's piece in the
							// first C_PASS clock, LW1's in the clock after
							// (st_chk2 / C_MST2); winv_set2 names LW1's set
							// for the fallback.
							r_row <= a_row;
							r_tag <= a_tag;
							r_addr <= c_addr;
							r_size <= c_size;
							r_off <= c_addr[1:0];
							r_wdata <= c_wdata;
							r_fc <= c_fc;
							r_cb <= (COPYBACK != 0) && c_cb && st_post_ok;
							if (MIS != 0) begin
								r_word <= {2'd0, c_addr[3:2]};
								mis_v <= st_mis_ok;
								mis_ph <= 1'b0;
								mis_two_r <= mis_two_c;
								mis_merge1 <= 1'b0;
								mis_row1 <= c_addr[9:4];
								mis_row2 <= {1'b0, mis_addr2_c[9:4]};
								mis_tag2 <= mis_addr2_c[31:10];
								mis_word2 <= mis_addr2_c[3:2];
								{r_be, mis_be2} <= win_be(c_size, c_addr[1:0]);
								{r_wd, mis_wd2} <= win_data(c_wdata, c_size, c_addr[1:0]);
								winv_set2 <= mis_addr2_c[9:4];
							end
							st_chk <= 1;
							st_flow <= 1;
							winv_pend <= 0;
							if (st_post_ok) begin
								// acknowledged next cycle; the drain
								// runs from the latched copy
								st_posted <= 1;
								ack_r <= 1;
							end
							cst <= C_PASS;
						end
						else begin
						// write-through.  Port B clears the set this store
						// touches in this acceptance cycle; a store crossing
						// the line owes a second one, taken during the pass
						// wait or in C_WINV.  The transfer itself is issued
						// from C_PASS (see pass_active); a posted store is
						// acknowledged from ack_r in the next cycle.
						winv_set2 <= c_addr[9:4] + 6'd1;
						winv_pend <= write_cross_line;
						mis_v <= 1'b0;
						r_addr <= c_addr;
						r_size <= c_size;
						r_wdata <= c_wdata;
						r_fc <= c_fc;
						if (st_post_ok) begin
							st_posted <= 1;
							ack_r <= 1;
						end
						cst <= C_PASS;
						end
					end
					else if (bypass) begin
						// A bypassed read needs the port, and it is a
						// serializing access: it waits for a draining
						// store, as the 68040 completes pending writes
						// before a serialized access.
						if (dr_active) begin
						end
						else begin
						// the tag row read runs in parallel here too, so
						// a cache-inhibited read can detect and kill a
						// resident line while it bypasses
						r_row <= a_row;
						r_tag <= a_tag;
						mis_v <= 1'b0;
						pass_ci_chk <= c_nocache && !c_write;
						cst <= C_PASS;
						end
					end
					else begin
						// cacheable read: the tag row read runs in parallel
						r_row <= a_row;
						r_tag <= a_tag;
						r_bank <= c_instr;
						r_addr <= c_addr;
						r_size <= c_size;
						r_off <= c_addr[1:0];
						r_word <= {2'd0, c_addr[3:2]};
						// (MIS) a misaligned read: LW0's lookup first
						mis_v <= mis_acc;
						mis_ph <= 1'b0;
						mis_two_r <= mis_two_c;
						mis_row1 <= c_addr[9:4];
						mis_row2 <= {1'b0, mis_addr2_c[9:4]};
						mis_tag2 <= mis_addr2_c[31:10];
						mis_word2 <= mis_addr2_c[3:2];
						mis_addr2 <= mis_addr2_c;
						cst <= C_LOOK;
					end
				end
			end

			C_PASS: begin
				// the second-set invalidate clears only when port B truly
				// served it; a snoop or a recorded first-set replay owns
				// the port this cycle and winv stays pending
				if (!s_stb && !store_inv_lost) winv_pend <= 0;
				if (pass_ci_chk) begin
					pass_ci_chk <= 0;
					if (look_hit) begin
						ci_inv_pend <= 1;
						ci_inv_row  <= r_row;
					end
				end
				if (st_chk) begin
					// the merge itself is st_merge (combinational, this
					// cycle).  A snooped window means the compare cannot
					// be trusted: fall back to the row invalidate through
					// the recorded slot -- free here, because a store is
					// accepted only while that slot is empty.
					st_chk <= 0;
					if (st_snooped) begin
						store_inv_lost <= 1;
						store_inv_set  <= r_row[5:0];
					end
					// (MIS) on to LW1: its row and word were read in this
					// clock (mis_launch2); the compare in the next clock is
					// against LW1's tag in LW1's row
					if (mis_v && mis_two_r) begin
						mis_ph <= 1'b1;
						mis_merge1 <= st_merge;
						r_row <= mis_row2;
						r_tag <= mis_tag2;
						r_word <= {2'd0, mis_word2};
						r_be <= mis_be2;
						r_wd <= mis_wd2;
					end
				end
				// (MIS) LW1's merge clock of a synchronous misaligned store
				// still waiting for its acknowledge: a snooped window drops
				// LW1's row (winv) instead of trusting the compare
				if (mis_m2 && st_snooped2 && !st_merge2) winv_pend <= 1;
				if (st_chk && mis_v && mis_two_r && st_posted && r_cb) begin
					// (MIS, COPYBACK) the memory write is decided by both
					// pieces' verdicts: in C_MST2, with LW1's
					cst <= C_MST2;
				end
				else if (st_posted && r_cb && st_merge) begin
					// (COPYBACK) the store hit a line of a copyback page:
					// it merged and the line is dirty; memory is not written
					st_posted <= 0;
					st_flow   <= 0;
					r_cb      <= 0;
					mis_v     <= 0;   // (MIS) a one-piece store: done
					cst <= C_IDLE;
				end
				else if (wa_go) begin
					// (WALLOC) the copyback store missed: allocate its line.
					// p_set names the row for the tag reads from here on;
					// C_LOOK takes its miss branch (the victim, the fill).
					st_posted <= 0;
					st_flow   <= 0;
					r_cb      <= 0;
					mis_v     <= 0;
					r_bank    <= 1'b0;
					p_set     <= r_row[5:0];
					wa_v      <= 1'b1;
					cst <= C_WAL;
				end
				else if (st_posted) begin
					r_cb <= 0;
					// A1-3a: the posted store is handed to the drain in
					// this, its first C_PASS cycle -- the merge above has
					// just landed -- and the FSM is free again.  A
					// line-crossing store's second-row invalidate, if port
					// B has not served it yet, is finished in C_WINV.
					dr_active <= 1;
					dr_addr   <= r_addr;
					dr_wdata  <= r_wdata;
					dr_size   <= r_size;
					dr_fc     <= r_fc;
					st_posted <= 0;
					st_flow   <= 0;
					// (MIS) LW1's merge still to come: C_MST2; a one-piece
					// store is done (mis_v is 0 whenever the FSM is idle)
					mis_v     <= st_chk && mis_v && mis_two_r;
					cst <= (st_chk && mis_v && mis_two_r) ? C_MST2 :
					       (winv_pend && (s_stb || store_inv_lost))
					       ? C_WINV : C_IDLE;
				end
				else if (m_err) begin
					// a passed access faulted: release the bus, but a
					// still-owed invalidate is honoured (invalidating
					// more is always safe under write-through)
					err_hold <= 1;
					if (st_flow) begin
						// the line may hold a store memory never took:
						// drop the row, the store restarts and re-merges
						// (MIS: LW0's row here, LW1's through winv)
						store_inv_lost <= 1;
						store_inv_set  <= (mis_v && mis_ph) ? mis_row1 : r_row[5:0];
						if (mis_v && mis_two_r) winv_pend <= 1;
					end
					// a posted store cannot restart: the core has already
					// been acknowledged and has moved on.  Report it as
					// the late error it is (the core halts).
					if (st_posted) post_err <= 1;
					st_posted <= 0;
					st_flow <= 0;
					mis_v <= 0;
					cst <= ((winv_pend && (s_stb || store_inv_lost)) || (st_flow && mis_v && mis_two_r))
					       ? C_WINV : C_IDLE;
				end
				else if (m_ack) begin
					st_posted <= 0;
					st_flow <= 0;
					// (MIS) a synchronous misaligned store acknowledged in
					// its first C_PASS clock: LW1's merge in C_MST2
					if (st_chk && mis_v && mis_two_r) cst <= C_MST2;
					else begin
						mis_v <= 0;
						cst <= (winv_pend && (s_stb || store_inv_lost))
						       ? C_WINV : C_IDLE;
					end
				end
			end

			C_FERR: begin
				// The core withdraws the faulting request while this
				// state runs, and only C_IDLE used to watch for that.
				// Release the hold here too, or a request raised again
				// before C_IDLE is reached is blocked forever.
				if (!c_req) err_hold <= 0;
				mis_v <= 0;   // (MIS) the two-piece request is abandoned whole
				// a free-running snoop owns port B when it fires; retry
				// until this row's invalidate is the one that lands
				if (!snoop_wr) cst <= C_IDLE;
			end

			C_WINV: begin
				if (!s_stb && !store_inv_lost) begin
					winv_pend <= 0;
					cst <= C_IDLE;
				end
			end

			C_WAL: begin
				// (WALLOC) the store's row is being read at p_set (tag_ridx);
				// the next clock's rq is the victim, its tags the row's
				cst <= C_LOOK;
			end

			C_MLNCH: begin
				// (MIS) a misaligned read's second piece: LW1's row and
				// word are read in this clock (mis_launch2 drives the
				// addresses from mis_row2 / mis_word2; r_* already name LW1);
				// the compare is C_LOOK's, as for any lookup
				cst <= C_LOOK;
			end

			C_MST2: begin
				// (MIS) a misaligned store's second piece, LW1: the merge
				// is st_merge2 in this clock (mis_m2).  A snooped window
				// drops LW1's row through winv instead.  A copyback store
				// whose memory write was deferred to this verdict stays in
				// the cache only if both pieces merged (both lines dirty);
				// otherwise it drains to memory as before (the merged piece,
				// if any, is in a line that is dirty or matches memory).
				if (st_snooped2 && !st_merge2) winv_pend <= 1;
				if (st_posted) begin
					st_posted <= 0;
					st_flow   <= 0;
					r_cb      <= 0;
					if (!(mis_merge1 && st_merge2 && r_cb)) begin
						dr_active <= 1;
						dr_addr   <= r_addr;
						dr_wdata  <= r_wdata;
						dr_size   <= r_size;
						dr_fc     <= r_fc;
					end
				end
				mis_v <= 0;
				cst <= (st_snooped2 && !st_merge2) ? C_WINV : C_IDLE;
			end

			C_SWEEP: begin
				if (sweep_push) begin
					// (COPYBACK) the row has dirty ways: push them (once the
					// drain is idle), then clear the row on the way back
					if (!dr_active && !walk_busy) begin
						p_set  <= sweep_cnt[5:0];
						p_mask <= db[sweep_cnt[5:0]];
						p_way  <= low_way(db[sweep_cnt[5:0]]);
						p_ph   <= 1'b0;
						p_rb   <= 3'd0;
						p_req  <= 1'b0;
						p_ret  <= P_SWEEP;
						cst    <= C_PUSH;
					end
				end
				else begin
				// one row per cycle; port A writes it (see sweep_hit)
				sweep_cnt <= sweep_cnt + 7'd1;
				if (sweep_cnt == 7'd127) begin
					if (!sweep_all) cinv_done <= 1;
					sweep_all <= 0;
					cst <= C_IDLE;
				end
				end
			end

			C_WLOOK: begin
				// the set's tags arrive the clock after p_set (wl_ph)
				if (!wl_ph)
					wl_ph <= 1'b1;
				else begin
					wk_chk <= 1'b1;
					if (wl_match != 4'd0) begin
						p_mask <= wl_match;
						p_way  <= low_way(wl_match);
						p_ph   <= 1'b0;
						p_rb   <= 3'd0;
						p_req  <= 1'b0;
						p_ret  <= P_IDLE;
						cst    <= C_PUSH;
					end
					else
						cst <= C_IDLE;
				end
			end

			C_PUSH: begin
				if (!p_ph) begin
					// read the way's four longwords: the address of
					// longword p_rb goes out, the one before arrives
					p_rb <= p_rb + 3'd1;
					case (p_rb)
						3'd1: pbuf0 <= p_dsel;
						3'd2: pbuf1 <= p_dsel;
						3'd3: pbuf2 <= p_dsel;
						3'd4: pbuf3 <= p_dsel;
						default: ;
					endcase
					if (p_rb == 3'd4) begin
						p_tag <= p_tsel;
						p_ph  <= 1'b1;
						p_wb  <= 2'd0;
						p_req <= 1'b1;
					end
				end
				else if (p_req && m_err) begin
					// a write-back cannot be restarted: the late error
					// the core halts on (post_err), and the push is over
					p_req    <= 1'b0;
					post_err <= 1'b1;
					p_mask   <= 4'd0;
					cst <= (p_ret == P_LOOK) ? C_LOOK : (p_ret == P_SWEEP) ? C_SWEEP : C_IDLE;
				end
				else if (p_req && m_ack) begin
					p_req <= 1'b0;
					if (p_wb == 2'd3) begin
						// the way is clean (db): the next dirty way, or back
						if ((p_mask & ~(4'd1 << p_way)) == 4'd0) begin
							p_mask <= 4'd0;
							cst <= (p_ret == P_LOOK) ? C_LOOK : (p_ret == P_SWEEP) ? C_SWEEP : C_IDLE;
						end
						else begin
							p_mask <= p_mask & ~(4'd1 << p_way);
							p_way  <= low_way(p_mask & ~(4'd1 << p_way));
							p_ph   <= 1'b0;
							p_rb   <= 3'd0;
						end
					end
					else p_wb <= p_wb + 2'd1;
				end
				else if (!p_req) p_req <= 1'b1;   // after the gap clock
			end

			C_LOOK: begin
				if (look_hit && ((!look_snooped && !snoop_look_row) || look_dirty) &&
				    !((WALLOC != 0) && wa_v)) begin
					// all four ways were read alongside the tags, so the
					// hit completes here: two cycles request-to-ack
					if ((MIS != 0) && mis_v && mis_two_r && !mis_ph) begin
						// (MIS) LW0 found: keep it, on to LW1 (its row and
						// word are read in C_MLNCH; a miss there fills
						// LW1's line through r_*)
						mis_lw0 <= data_hit;
						mis_ph  <= 1'b1;
						r_row   <= mis_row2;
						r_tag   <= mis_tag2;
						r_addr  <= mis_addr2;
						r_word  <= {2'd0, mis_word2};
						cst <= C_MLNCH;
					end
					else begin
						rdata_r <= ((MIS != 0) && mis_v) ? lw_extract2(mis_lw0, data_hit, r_size, r_off, mis_two_r)
						                                 : lw_extract(data_hit, r_size, r_off);
						ack_r <= 1;
						mis_v <= 0;
						cst <= C_IDLE;
					end
				end
				else begin
					r_way <= rq;   // round-robin victim
					r_beat <= 0;
					r_issued <= 0;
					if (dr_active) begin
						// a miss waits here for the draining store: the
						// tag row keeps being read at the held address, a
						// snoop meanwhile forces the miss it would have
						// forced anyway, and memory order is preserved
						// (the fill cannot overtake the store it may be
						// reading)
					end
					else if ((COPYBACK != 0) && !r_row[6] && db[r_row[5:0]][rq]) begin
						// (COPYBACK) the victim is dirty: push it, then
						// come back here (still a miss, now a clean victim);
						// not while a table walk owns the master port
						if (!walk_busy) begin
						p_set  <= r_row[5:0];
						p_mask <= 4'd1 << rq;
						p_way  <= rq;
						p_ph   <= 1'b0;
						p_rb   <= 3'd0;
						p_req  <= 1'b0;
						p_ret  <= P_LOOK;
						cst    <= C_PUSH;
						end
					end
					else if (FILL_CHANNEL != 0 && fill_ok) begin
						// A1: the whole line over the fill channel
						fill_req <= 1;
						cst <= C_FILLC;
					end
					else cst <= C_FILL;
				end
			end

			C_FILLC: begin
				// The channel answers with the whole line, or with an
				// error.  Nothing has touched the victim way yet, so an
				// error leaves nothing to clean up; C_FERR is taken
				// anyway so the abandoned request is released on the
				// one path that already knows how (err_hold, the row
				// invalidate is idempotent).
				if (fill_err) begin
					fill_req <= 0;
					// (WALLOC) the allocate's fill: the core holds no request
					// of its own to release; the posted store is lost, the
					// late error the core cannot restart
					if ((WALLOC != 0) && wa_v) begin post_err <= 1'b1; wa_v <= 1'b0; end
					else err_hold <= 1;
					cst <= C_FERR;
				end
				else if (fill_ack) begin
					fill_req <= 0;
					fill_line <= fill_data;
					r_beat <= 0;
					cst <= C_FILLW;
				end
			end

			C_FILLW: begin
				// one longword per cycle into the victim way (cd_we);
				// the tag write in C_TAGW validates the line, or not, by
				// the same fill_snooped rule as the adapter path
				if (r_beat == r_addr[3:2]) fill_hold <= fill_beat;
				if (r_beat == 2'd3) cst <= C_TAGW;
				else r_beat <= r_beat + 2'd1;
			end

			C_FILL: begin
				if (m_err) begin
					// 5.4: abandon the fill.  The incoming line is
					// never validated (only C_TAGW validates it), but
					// the beats already written landed in the VICTIM
					// way, whose old tag and valid bit are still live
					// -- so the evicted line would keep hitting over
					// corrupted data.  C_FERR invalidates the row
					// before anything can look at it.  err_hold keeps
					// the still-asserted request from being
					// re-accepted before the core withdraws it.
					r_issued <= 0;
					if ((WALLOC != 0) && wa_v) begin post_err <= 1'b1; wa_v <= 1'b0; end   // (WALLOC) as C_FILLC
					else err_hold <= 1;
					cst <= C_FERR;
				end
				else if (!r_issued) r_issued <= 1;
				else if (m_ack) begin
					// the data RAM write runs in parallel (cd_we)
					if (r_beat == r_addr[3:2]) fill_hold <= m_rdata;
					r_issued <= 0;
					if (r_beat == 2'd3) cst <= C_TAGW;
					else r_beat <= r_beat + 2'd1;
				end
			end

			C_TAGW: begin
				// the tag row write runs in parallel (tag_we): new tag,
				// its valid bit, and the advanced round robin
				if ((MIS != 0) && mis_v && mis_two_r && !mis_ph) begin
					// (MIS) LW0's line filled: keep LW0, on to LW1 (the tag
					// row write of this clock uses the old r_row, LW0's)
					mis_lw0 <= fill_hold;
					mis_ph  <= 1'b1;
					r_row   <= mis_row2;
					r_tag   <= mis_tag2;
					r_addr  <= mis_addr2;
					r_word  <= {2'd0, mis_word2};
					cst <= C_MLNCH;
				end
				else if ((WALLOC != 0) && wa_v) begin
					// (WALLOC) the merge (cd_we, wa_ok) and the dirty bit land
					// in this clock with the valid bit, or none of them does:
					// then the store drains to memory as a write-through
					wa_v <= 1'b0;
					if (!wa_ok) begin
						dr_active <= 1;
						dr_addr   <= r_addr;
						dr_wdata  <= r_wdata;
						dr_size   <= r_size;
						dr_fc     <= r_fc;
					end
					cst <= C_IDLE;
				end
				else begin
					rdata_r <= ((MIS != 0) && mis_v) ? lw_extract2(mis_lw0, fill_hold, r_size, r_off, mis_two_r)
					                                 : lw_extract(fill_hold, r_size, r_off);
					ack_r <= 1;
					mis_v <= 0;
					cst <= C_IDLE;
				end
			end

			default: cst <= C_IDLE;
		endcase
	end
end

//---------------------------------------------------------------------------
// instruction read path (plan M14, IFP = 1)
//---------------------------------------------------------------------------
// The FSM above serves one request at a time and answers a hit two clocks
// after accepting it, behind a registered request and ahead of a
// request-low clock: four clocks a transfer, and the pipelined core's IF
// needs a longword every clock (PLAN M11.2).  This is a separate read-only
// copy of the INSTRUCTION bank -- its tags, its data and (in flops) its
// valid bits -- written by exactly the writes the main arrays take for
// instruction-bank rows, and read by its own port: the address is taken at
// the enabled edge the requester launches, the answer is decided on the
// block-RAM outputs in the following clock.  Nothing the FSM does changes:
// fills, invalidates, the CINV sweep, the snoop and every store rule are
// the verified machinery and remain the only writers.  The copy can answer
// "hit" only for a line the main arrays hold as well, with two exceptions
// handled here:
//   * a fill writes the victim way's DATA before its tag, so the way being
//     filled is masked for the whole fill (C_FILL/C_FILLC/C_FILLW/C_TAGW/
//     C_FERR) and for the clock after it (the tag copy's read in the edge
//     that writes it may return either tag);
//   * a row invalidate (port B) clears the copy's valid bits in the same
//     edge, and wins over a port-A write to the same row.
// The instruction bank is never snooped and never written by a store (the
// 68040 does not snoop its I-cache on CPU writes), so fills and invalidates
// are its only writers.
generate
if (IFP != 0) begin : g_ifp
	(* ramstyle = "no_rw_check" *) reg [87:0] itag  [0:63];
	(* ramstyle = "no_rw_check" *) reg [31:0] idat0 [0:255];
	(* ramstyle = "no_rw_check" *) reg [31:0] idat1 [0:255];
	(* ramstyle = "no_rw_check" *) reg [31:0] idat2 [0:255];
	(* ramstyle = "no_rw_check" *) reg [31:0] idat3 [0:255];
	reg  [87:0] itag_q;
	reg  [31:0] idat_q0, idat_q1, idat_q2, idat_q3;
	reg   [5:0] f_set;
	always @(posedge clk) begin
		if (ce & tag_we & tag_widx[6]) itag[tag_widx[5:0]] <= tag_wdat[87:0];
		if (ce & cd_we[0] & cd_widx[8]) idat0[cd_widx[7:0]] <= cd_wdat;
		if (ce & cd_we[1] & cd_widx[8]) idat1[cd_widx[7:0]] <= cd_wdat;
		if (ce & cd_we[2] & cd_widx[8]) idat2[cd_widx[7:0]] <= cd_wdat;
		if (ce & cd_we[3] & cd_widx[8]) idat3[cd_widx[7:0]] <= cd_wdat;
		if (ce & ifp_en) begin
			itag_q  <= itag[ifp_addr[9:4]];
			idat_q0 <= idat0[ifp_addr[9:2]];
			idat_q1 <= idat1[ifp_addr[9:2]];
			idat_q2 <= idat2[ifp_addr[9:2]];
			idat_q3 <= idat3[ifp_addr[9:2]];
			f_set   <= ifp_addr[9:4];
		end
	end
	// the valid bits are the main lookup's (vb): the same writes
	// the way a fill is overwriting, and the clock after the fill ends
	wire fill_st = (cst == C_FILL) || (cst == C_FILLC) || (cst == C_FILLW) ||
	               (cst == C_TAGW) || (cst == C_FERR);
	reg        fm_v;
	reg  [5:0] fm_set;
	reg  [1:0] fm_way;
	always @(posedge clk) begin
		if (!nreset) fm_v <= 1'b0;
		else begin
			fm_v   <= fill_st && r_row[6];
			fm_set <= r_row[5:0];
			fm_way <= r_way;
		end
	end
	wire [3:0] mask_now  = (fill_st && r_row[6] && r_row[5:0] == f_set) ? (4'd1 << r_way) : 4'd0;
	wire [3:0] mask_prev = (fm_v && fm_set == f_set) ? (4'd1 << fm_way) : 4'd0;
	wire [3:0] vv = vb[{1'b1, f_set}] & ~mask_now & ~mask_prev;
	wire g0 = vv[0] && (itag_q[21:0]  == ifp_ptag);
	wire g1 = vv[1] && (itag_q[43:22] == ifp_ptag);
	wire g2 = vv[2] && (itag_q[65:44] == ifp_ptag);
	wire g3 = vv[3] && (itag_q[87:66] == ifp_ptag);
	assign ifp_thit  = g0 | g1 | g2 | g3;
	assign ifp_rdata = g0 ? idat_q0 : g1 ? idat_q1 : g2 ? idat_q2 : idat_q3;
end else begin : g_noifp
	assign ifp_thit  = 1'b0;
	assign ifp_rdata = 32'd0;
	wire unused_ifp = ifp_en | (|ifp_addr) | (|ifp_ptag);
end
endgenerate

//---------------------------------------------------------------------------
// data read path (plan M14 step 3, IFP = 1)
//---------------------------------------------------------------------------
// The instruction path's copy, for the DATA bank.  The data bank has more
// writers than the instruction bank -- stores merge into it (st_merge) or
// clear a row (port B), and chipset DMA snoops invalidate rows (port B, not
// clock-enabled) -- so besides mirroring every one of them, a lookup is
// refused when any write reaches its SET in the edge that launched it (the
// copy's read and write collide) or in the answer clock (a snoop landing
// while the answer is taken).  Store ORDER is the requester's business: the
// core takes an answer only with no older store in flight.
generate
if (IFP != 0) begin : g_dfp
	(* ramstyle = "no_rw_check" *) reg [87:0] dtag  [0:63];
	(* ramstyle = "no_rw_check" *) reg [31:0] ddat0 [0:255];
	(* ramstyle = "no_rw_check" *) reg [31:0] ddat1 [0:255];
	(* ramstyle = "no_rw_check" *) reg [31:0] ddat2 [0:255];
	(* ramstyle = "no_rw_check" *) reg [31:0] ddat3 [0:255];
	reg  [87:0] dtag_q;
	reg  [31:0] ddat_q0, ddat_q1, ddat_q2, ddat_q3;
	reg   [5:0] g_set;
	reg   [1:0] g_size, g_off;
	reg         g_col;
	// (DFP_MIS) a misaligned read spanning two longwords (plan step 3).
	// Clock A (dfp_en): the first longword is read as any lookup's.  Clock B
	// (g_two): its verdict is taken as today (dfp_thit) and kept (g_h1, its
	// data g_d1, its set and way g_set1/g_way1, the physical tag of the
	// second longword g_ptag2), while the copy's one read port fetches the
	// second longword (g_idx2: the next longword of the bank, +1 on the
	// 8-bit index; the tag is incremented when the index wraps the 1K
	// bank -- never past the 4K page, which two_in excludes).  Clock C
	// (g_m2): the second longword's compare, the first's line re-checked
	// (still valid, no write to its set now), the operand assembled from the
	// 64-bit window.  The requester never launches a lookup in clock B (one
	// read outstanding); if it did, the new lookup wins and the old one is
	// dropped (dfp_two low, g_m2 never set).
	reg         g_two;       // the lookup launched at the last edge needs a second (this clock)
	reg   [7:0] g_idx2;      // ... the second longword's index in the bank
	reg         g_m2;        // the second lookup was launched at the last edge: the answer is now
	reg  [21:0] g_ptag2;     // the second longword's physical tag
	reg   [5:0] g_set1;      // the first longword's set, way (one-hot), data and verdict
	reg   [3:0] g_way1;
	reg  [31:0] g_d1;
	reg         g_h1;
	wire        two_in = (DFP_MIS != 0) && (dfp_addr[11:2] != 10'h3FF) &&
	                     ((dfp_size == `AP040_SZ_L && dfp_addr[1:0] != 2'b00) ||
	                      (dfp_size == `AP040_SZ_W && dfp_addr[1:0] == 2'b11));
	wire        rd2    = (DFP_MIS != 0) && g_two && !dfp_en;
	wire  [7:0] r_idx  = rd2 ? g_idx2 : dfp_addr[9:2];
	wire  [5:0] l_set  = r_idx[7:2];
	// a write of the data bank's set l_set in this edge
	wire w_tag = ce & tag_we & !tag_widx[6];
	wire w_inv = inv_wren & !inv_idx[6];
	wire w_dat = ce & (|cd_we) & !cd_widx[8];
	// (declared ahead: the first longword's verdict and data, kept at the edge ending clock B)
	wire        thit_now;
	wire [31:0] dsel;
	wire        e0, e1, e2, e3;
	always @(posedge clk) begin
		if (w_tag) dtag[tag_widx[5:0]] <= tag_wdat[87:0];
		if (ce & cd_we[0] & !cd_widx[8]) ddat0[cd_widx[7:0]] <= cd_wdat;
		if (ce & cd_we[1] & !cd_widx[8]) ddat1[cd_widx[7:0]] <= cd_wdat;
		if (ce & cd_we[2] & !cd_widx[8]) ddat2[cd_widx[7:0]] <= cd_wdat;
		if (ce & cd_we[3] & !cd_widx[8]) ddat3[cd_widx[7:0]] <= cd_wdat;
		if (ce & (dfp_en | rd2)) begin
			dtag_q  <= dtag[l_set];
			ddat_q0 <= ddat0[r_idx];
			ddat_q1 <= ddat1[r_idx];
			ddat_q2 <= ddat2[r_idx];
			ddat_q3 <= ddat3[r_idx];
			g_set   <= l_set;
			g_col   <= (w_tag && tag_widx[5:0] == l_set) ||
			           (w_inv && inv_idx[5:0] == l_set) ||
			           (w_dat && cd_widx[7:2] == l_set);
		end
		if (ce & dfp_en) begin
			g_size  <= dfp_size;
			g_off   <= dfp_addr[1:0];
			g_idx2  <= dfp_addr[9:2] + 8'd1;
		end
		if (!nreset) begin
			g_two <= 1'b0; g_m2 <= 1'b0;
		end else if (ce) begin
			g_two <= dfp_en && two_in;
			g_m2  <= rd2;
			if (rd2) begin
				g_h1    <= thit_now;
				g_d1    <= dsel;
				g_set1  <= g_set;
				g_way1  <= {e3, e2, e1, e0};
				// the tag plus one when the index wrapped the 1K bank: a
				// full 22-bit increment (two_in keeps the carry inside
				// pa[11:10] today, but the sum does not depend on it)
				g_ptag2 <= dfp_ptag + {21'd0, (g_idx2 == 8'd0)};
			end
		end
	end
	// the valid bits are the main lookup's (vb): the same writes
	wire fill_d = ((cst == C_FILL) || (cst == C_FILLC) || (cst == C_FILLW) ||
	               (cst == C_TAGW) || (cst == C_FERR)) && !r_row[6];
	reg        dm_v;
	reg  [5:0] dm_set;
	reg  [1:0] dm_way;
	always @(posedge clk) begin
		if (!nreset) dm_v <= 1'b0;
		else begin
			dm_v   <= fill_d;
			dm_set <= r_row[5:0];
			dm_way <= r_way;
		end
	end
	wire [3:0] dmask_now  = (fill_d && r_row[5:0] == g_set) ? (4'd1 << r_way) : 4'd0;
	wire [3:0] dmask_prev = (dm_v && dm_set == g_set) ? (4'd1 << dm_way) : 4'd0;
	wire       d_busy_now = (w_inv && inv_idx[5:0] == g_set) ||
	                        (w_tag && tag_widx[5:0] == g_set) ||
	                        (w_dat && cd_widx[7:2] == g_set);
	wire [3:0] dvv = vb[{1'b0, g_set}] & ~dmask_now & ~dmask_prev;
	// (DFP_MIS) the compare runs against both candidate tags -- the
	// requester's for a lookup answered now, the kept second-longword tag
	// in the misaligned read's third clock -- and the choice is made after
	// it, so the compare path is no deeper than before.
	wire t0 = (dtag_q[21:0]  == dfp_ptag), t1 = (dtag_q[43:22] == dfp_ptag);
	wire t2 = (dtag_q[65:44] == dfp_ptag), t3 = (dtag_q[87:66] == dfp_ptag);
	wire m0 = (DFP_MIS != 0) && (dtag_q[21:0]  == g_ptag2), m1 = (DFP_MIS != 0) && (dtag_q[43:22] == g_ptag2);
	wire m2 = (DFP_MIS != 0) && (dtag_q[65:44] == g_ptag2), m3 = (DFP_MIS != 0) && (dtag_q[87:66] == g_ptag2);
	assign e0 = dvv[0] && (g_m2 ? m0 : t0);
	assign e1 = dvv[1] && (g_m2 ? m1 : t1);
	assign e2 = dvv[2] && (g_m2 ? m2 : t2);
	assign e3 = dvv[3] && (g_m2 ? m3 : t3);
	// (findings/storebuf/plan.md) not while a store still owes an
	// invalidate: its second line's (winv) or one a snoop displaced
	// (store_inv_lost).  The data bank may hold the line the store changed.
	assign thit_now = (e0 | e1 | e2 | e3) && !g_col && !d_busy_now &&
	                  !store_inv_lost && !winv_pend && (cst != C_WINV);
	assign dsel     = e0 ? ddat_q0 : e1 ? ddat_q1 : e2 ? ddat_q2 : ddat_q3;
	// (DFP_MIS) the first longword in the third clock: its line still valid
	// (a snoop may have landed on any clock edge since), not being filled,
	// and no write reaching its set at this edge
	wire [3:0] dmask1_now  = (fill_d && r_row[5:0] == g_set1) ? (4'd1 << r_way) : 4'd0;
	wire [3:0] dmask1_prev = (dm_v && dm_set == g_set1) ? (4'd1 << dm_way) : 4'd0;
	wire       d1_busy     = (w_inv && inv_idx[5:0] == g_set1) ||
	                         (w_tag && tag_widx[5:0] == g_set1) ||
	                         (w_dat && cd_widx[7:2] == g_set1);
	wire       h1_ok       = (DFP_MIS != 0) && g_h1 && !d1_busy &&
	                         (|(vb[{1'b0, g_set1}] & ~dmask1_now & ~dmask1_prev & g_way1));
	assign dfp_thit  = thit_now && (!g_m2 || h1_ok);
	assign dfp_two   = (DFP_MIS != 0) && g_two;
	assign dfp_rdata = lw_extract2(g_d1, dsel, g_size, g_off, g_m2);
end else begin : g_nodfp
	assign dfp_thit  = 1'b0;
	assign dfp_two   = 1'b0;
	assign dfp_rdata = 32'd0;
	wire unused_dfp = dfp_en | (|dfp_addr) | (|dfp_size) | (|dfp_ptag);
end
endgenerate

endmodule
