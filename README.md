# AP68040-pipelined

A pipelined MC68040-compatible CPU core in Verilog/SystemVerilog, with MMU,
FPU and split instruction/data caches, integrated into
[MinimigAGA_TC64](https://github.com/ranzbak/MinimigAGA_TC64) (QMTech
XC7A100T). It is a fork of [AP68040](https://github.com/nonarkitten/AP68040):
the pipelined branch completed, verified and brought up on hardware.

## Credits

- **Adam Polkosnik** wrote AP68040, the original sequential 68040 core with its
  MMU, FPU, caches and test benches, as part of
  [Minimig-AGA_MiSTer](https://github.com/apolkosnik/Minimig-AGA_MiSTer). He
  holds the copyright on that work, and most of `rtl/compat/` and `rtl_old/`
  is his.
- **Renee Cousins (nonarkitten)** started the six-stage pipelined core in
  `rtl/`.
- **Paul Honig** completed the pipelined core (the full integer ISA, exceptions,
  MMU, FPU, split read paths and timing closure) and its Minimig integration.

## Status

- Six stages, as on the real 68040: IF, ID, EA-calc, EA-fetch, EX, WB.
- Hardware (QMTech XC7A100T, MinimigAGA_TC64): boots Kickstart 3.1.4 to
  Workbench, and cputest `ct040_01` passes. The wider board test campaign is
  still in progress.
- Simulation: `tb/run_pipe_tests.sh` passes all legs with the board's switches
  (324, or 328 with `PIPE_COPYBACK=1`; October 2026). WinUAE cputest
  Basic 670/670 plus six further groups. Kickstart 3.1.4 boots in the
  differential bench against the original core.
- Split instruction/data read paths (plan M14): a cache hit is answered the
  clock after it is issued. Measured at about 2.1x the previous throughput on
  SysInfo's loop in the SoC bench.
- FPU: the 68040's hardware subset. Unimplemented instructions trap as on a
  real 68040 and need an FPSP (for example `68040.library`).

The milestone log, open questions and every deviation from the manual are in
`AP040_IMPLEMENTATION_PLAN.md` and `TIMING-LOG.md`.

## Performance

Measured on the board (QMTech XC7A100T in MinimigAGA_TC64), with the core
clocked at **37.8125 MHz** (`clk_114/3`), the MMU on under MMULib's
`68040.library`, the FPU on, and the Minimig's fast RAM (Zorro II/III SDRAM
and the DDR3 board). The benchmark is
[xSysInfo](https://github.com/reinauer/xSysInfo)'s Dhrystone 2.1, which rates
against an A4000/040 at 25 MHz.

| Core | Dhrystones/s | vs 68040 @ 25 MHz |
| --- | ---: | ---: |
| before the store buffer and branch work | 21,721 | 0.66 |
| + store buffer, store-to-load forwarding, return-address stack | 22,185 | 0.67 |
| + address-precise fast reads | 28,938 | 0.88 |
| + stores posted with the MMU on | 33,287 | 1.01 |
| + forward branches guessed not taken (release 0.1) | 36,256 | 1.10 |
| + late-operand load dispatch | 37,316 | 1.13 |
| + copyback data cache (DDR3 fast RAM set to CopyBack) | 41,656 | 1.26 |
| **+ misaligned accesses served from the data cache (current)** | **47,413** | **1.44** |

That is about 1,250 Dhrystones/s per MHz (0.71 DMIPS/MHz), against about
1,320 for a real 68040: within about 5 % of the real chip per clock, and
ahead of a 25 MHz 68040 by running at 38 MHz. The last two rows need the
fast RAM marked CopyBack in the MMU configuration (MMULib); in
write-through the copyback hardware stores through like the earlier rows.

The features behind the table are parameters of `ap040_pipe_tg68k_compat`,
all 0 (off) by default:

| Parameter | What it does | MinimigAGA_TC64 |
| --- | --- | --- |
| `AP040_STORE_BUF` | posts stores that cannot fault into a 4-entry FIFO (1) | 1 |
| `AP040_FWD` | a load an older store covers is answered from that store | 1 |
| `AP040_RAS` | an 8-entry return-address stack predicts RTS in ID | 1 |
| `AP040_PRECISE` | fast reads compare addresses with the stores in flight, instead of waiting for all of them | 1 |
| `AP040_SB_MMU` | with translation on, stores to a page a synchronous store proved writable are posted too | 1 |
| `AP040_BTFN` | ID guesses a forward conditional branch not taken | 1 |
| `AP040_LDX` | an eligible load goes to EX in the clock it is looked up | 1 |
| `AP040_COPYBACK` | the data cache keeps stores to CM = 01 pages (copyback), DDR3 board only | 1 |
| `AP040_DFP_MIS` | a misaligned access inside a 4K page is served from (or fills, or merges into) the data cache | 1 |
| `AP040_WALLOC` | a one-piece store that misses a CM = 01 page fills its line and stays in the cache, dirty (write-allocate; needs `AP040_COPYBACK`) | 0 |
| `AP040_BTB` | IF's branch target buffer | 0 |
| `AP040_MISPLIT` | splits misaligned transfers into aligned pieces | 0 |

With the MMU on, `AP040_SB_MMU` posts only into the DDR3 board's window
(`mem_postok` in `ap040_pipe_tg68k_compat.v`): posting into the SDRAM as well
corrupted data on the board while every simulation passed, and that cause is
still open. `AP040_MISPLIT` gains nothing on the board and is not used;
`AP040_DFP_MIS` replaces it.

## What is here

```
rtl/                    the pipelined core
  ap040_pipe_core.v       top of the pipeline: IF/ID/EA-calc/EA-fetch/EX/WB
  ap040_inst_fetch.v  ap040_decode.v  ap040_ea_calc.v  ap040_ea_fetch.v
  ap040_execute.v  ap040_writeback.v      the six stages
  ap040_pipe_bcu.v        bus control unit (the shared memory port)
  ap040_pipe_alu.v  ap040_pipe_muldiv.v  ap040_pipe_regfile.v
  ap040_pipe_pkg.sv  ap040_pipe_defs.svh  shared types and defines
  compat/                 the host-facing shell around the pipeline
    ap040_pipe_tg68k_compat.v   TOP LEVEL: TG68K-shaped port set
    ap040_cache.v  ap040_mmu.v  ap040_fpu.v   caches, MMU, FPU
    ap040_bus16_adapter.v  ap040_bus_timeout.v  ap040_walker_cdc.v
    ap040_defs.svh  ap040_fpu_tie.vh  primitives/
rtl_old/                the original sequential core (reference only)
tb/                     standalone test benches for both cores
doc/
```

## Integrating it

**Files.** Add `rtl/ap040_pipe_pkg.sv` first, then every `rtl/ap040_*.v` and
every `rtl/compat/*.v` (plus `rtl/compat/primitives/*.v`), all as
SystemVerilog, with `rtl/` and `rtl/compat/` on the include path. The Quartus
`rtl/ap040_pipe.qip` is out of date; use the file list above, or copy
`tools/vivado/build_ap040.tcl` from MinimigAGA_TC64, which does exactly this
when `AP040_PIPE_DIR` points at this repository. Don't add `rtl_old/` to
the same project: some module names clash.

**Top level: `ap040_pipe_tg68k_compat`.**

```verilog
ap040_pipe_tg68k_compat #(
    .AP040_HAS_MMU     (1),   // 0: LC040 without MMU
    .AP040_HAS_FPU     (0),   // 1: FPU on (about +11k LUTs on Artix-7)
    .AP040_ENABLE_CACHE(1),
    .AP040_IFP         (1),   // M14 fast read paths (needs the cache)
    .AP040_BUS16       (0)    // 0: 32-bit m_* master port; 1: 16-bit host bus
) cpu (
    .clk (clk_cpu), .nreset (nreset), .clkena_in (1'b1),
    .ipl (ipl), .ipl_autovector (1'b1), .berr (berr), .nresetout (nresetout),
    .m_req (m_req), .m_write (m_write), .m_instr (m_instr), .m_size (m_size),
    .m_addr (m_addr), .m_wdata (m_wdata), .m_fc (m_fc),
    .m_ack (m_ack), .m_rdata (m_rdata),
    ...
);
```

**Port groups.**
- **`m_*`** (with `AP040_BUS16=0`): one 32-bit request at a time, fields
  stable while `m_req` is high, answered by `m_ack`. This is the mode
  MinimigAGA_TC64 uses, where `rtl/soc/TG68K.vhd` bridges it to SDRAM, DDR3
  and the chipset across clock domains. With `AP040_BUS16=1` the core talks
  to a TG68K-style 16-bit bus (`data_in`, `addr_out`, `busstate`, ...)
  instead.
- **`walker_*`**: the MMU table walker's own memory port. A walk that never
  gets `walker_ack` becomes a bus error; it never hangs the core.
- **`cache_snoop_*`**: DMA write snoop, already in the core's clock domain.
  Without it the data cache can't see writes from other bus masters.
  `cache_z2_*`/`cache_z3_*` describe the cacheable windows, and
  `cache_allow_all` bypasses them for flat simulation.
- **`fill_*`, `cache_maint_*`, `cache_req/…`**: line fills and cache
  maintenance, for a wrapper that serves them. Tie them off otherwise.
- **`mmu_*`, `cacr_out`, `vbr_out`, `debug_*`**: observation only.

**Things to get right.**
- It is a **restart-model** 68040: a faulting write is re-executed after the
  handler fixes the mapping, and the access-error frame never advertises a
  valid WB3.
- Timing on Artix-7 (speed grade -2) is closed at 38 MHz (`clk_114/3` in the
  Minimig build) with the FPU on. The critical paths are the memory
  acknowledge into the pipeline's stall chain; see `TIMING-LOG.md`.
- `primitives/dpram.v` is an inferred true-dual-port RAM. Swap in a vendor
  macro if your flow needs one.

## Testing

```
cd tb && ./run_pipe_tests.sh     # the pipelined core; needs iverilog only
cd tb && ./run_tests.sh          # rtl_old; also needs vasmm68k_mot (vbcc)
```

Every bench runs against the core alone, so a failure is the CPU's, not an
integration artifact. The benches that co-simulate the core with the Amiga
chipset, the SDRAM and DDR3 controllers, Kickstart, and the WinUAE cputest
corpus live in MinimigAGA_TC64 (`sim/ddr3_cpu`), because they need those
modules. vvp is single-threaded, so independent benches can run in parallel.

Order of authority when sources disagree: the M68040 User's Manual text, then
a source verified on real 68040 hardware for that case (such as WinUAE's
cputest-validated 040 code), then the Programmer's Reference Manual. Each
decision is recorded in the plan's divergences table.

## No warranty

This core is provided **as is**, without warranty of any kind, express or
implied, including fitness for a particular purpose. The authors take no
responsibility for any damage or loss resulting from its use, including to
hardware, software or data. It is experimental and still under test. Use it
at your own risk. See sections 15 and 16 of the GPL.

## License

GPL v3 or later; see [LICENSE](LICENSE).

Copyright © 2025-2026 Adam Polkosnik (AP68040)
Pipelined core © 2026 Renee Cousins, Paul Honig
