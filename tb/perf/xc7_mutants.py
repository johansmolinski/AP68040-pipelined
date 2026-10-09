#!/usr/bin/env python3
"""xc7_mutants.py [-j N] [-o out-dir] [-a] [-n] [names...]: the AP040_XC7
mutant table -- the data cache's valid, round-robin and dirty tables in
distributed RAM (rtl/compat/ap040_xc7_ram2w.v, ap040_cache.v g_vbram and
g_dbram).  Each mutant is a one-line edit of a scratch copy of rtl/.  It
runs first on tb_ap040_cache_snoop.v (the cache alone, -DAP040_PIPE_CACHE
-DXC7=1: snoops against tag writes, swept), then on the compat bench with
the board switches plus XC7=1, the store-order checker, tb_dfpmis_check.v
and tb_xc7_check.v (the shadow flip-flop tables) on cache_asm/
t_cbsnp_pipe.s, t_cbwa_pipe.s, t_cb_pipe.s, t_dfpmis_pipe.s and
t_icache_pipe.s.  A mutant is CAUGHT when a run does not print ALL TESTS
PASSED; the runs go in that order and stop at the first catch (-a: run
them all).  -n: the compat bench WITHOUT tb_xc7_check.v (what the programs
alone see).  Runs -j mutants in parallel (vvp is single-threaded; default
3).  Rows marked "equiv" are argued equivalent (expected to survive; the
reason is in the row)."""
import os, subprocess, sys, shutil
from concurrent.futures import ThreadPoolExecutor

T = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))     # tb/
R0 = os.path.join(os.path.dirname(T), "rtl")
DEFS = "-DSTORE_BUF=1 -DFWD=1 -DRAS=1 -DPRECISE=1 -DSB_MMU=1 -DLDX=1 -DBTFN=1 -DCOPYBACK=1 -DDFP_MIS=1 -DWALLOC=1 -DXC7=1".split()
PROGS = ["snoop", "t_cbsnp_pipe", "t_cbwa_pipe", "t_cb_pipe", "t_dfpmis_pipe", "t_icache_pipe"]
C = "compat/ap040_cache.v"
X = "compat/ap040_xc7_ram2w.v"

# name: (what it breaks, [(file, old, new), ...][, "equiv: why"])
MUT = {
 "x1_bank": ("port 1's XOR reads its own bank instead of bank 0 (a wrong bank select)", [
    (X, "if (we1) b1[a1] <= d1 ^ b0[a1];", "if (we1) b1[a1] <= d1 ^ b1[a1];")]),
 "x2_collision": ("both ports write on a same-row collision (port 1 no longer wins)", [
    (X, "if (we0 && !same) b0[a0] <= d0 ^ b1[a0];", "if (we0) b0[a0] <= d0 ^ b1[a0];")],
    "equiv in the cache: a tag write and a row clear never name the same row in one clock "
    "(snoop_fill_row / the sweep's snoop_wr guard drop the snoop's side; the ce-side clears are in other states)"),
 "x3_read_bank": ("the lookup's read takes bank 0 only", [
    (X, "assign rd[g*DW +: DW] = b0[a] ^ b1[a];", "assign rd[g*DW +: DW] = (g == 0) ? b0[a] : b0[a] ^ b1[a];")]),
 "x4_ports": ("vb's ports swapped: the tag write on port 1, the row clear on port 0", [
    (C, ".we0(ce & tag_we), .a0(tag_widx), .d0(tag_wdat[91:88]),\n\t\t.we1(inv_wren),    .a1(inv_idx),  .d1(vb_inv & vb_keep),",
        ".we1(ce & tag_we), .a1(tag_widx), .d1(tag_wdat[91:88]),\n\t\t.we0(inv_wren),    .a0(inv_idx),  .d0(vb_inv & vb_keep),")],
    "equiv in the cache, as x2: the two never name the same row in one clock"),
 "x5_rdorder": ("the DFP path's two valid-bit reads swapped (read port order)", [
    (C, ".ra({vb_ra_g1, vb_ra_g, vb_ra_f, inv_idx, vb_ra_q}),", ".ra({vb_ra_g, vb_ra_g1, vb_ra_f, inv_idx, vb_ra_q}),")]),
 "x6_nokeep": ("a row clear drops the dirty ways too (vb_keep ignored)", [
    (C, ".d1(vb_inv & vb_keep),", ".d1(4'd0),")]),
 "x7_vb_nosweep": ("the reset sweep does not clear the RAM's valid bits (a warm reset keeps the old lines)", [
    (C, ".we0(ce & tag_we), .a0(tag_widx), .d0(tag_wdat[91:88]),", ".we0(ce & tag_we & !sweep_all), .a0(tag_widx), .d0(tag_wdat[91:88]),")]),
 "x8_db_nosweep": ("the reset sweep does not clear the dirty table (a warm reset keeps the old dirty bits)", [
    (C, "wire       dw_swp = (cst == C_SWEEP) && sweep_all && !sweep_cnt[6];", "wire       dw_swp = 1'b0;")]),
 "x9_db_pushrow": ("the push's dirty-bit clear written at the looked-up row, not the pushed set", [
    (C, "wire [5:0] dw_row = (cst == C_PUSH)  ? p_set :", "wire [5:0] dw_row = (cst == C_PUSH)  ? r_row[5:0] :")]),
 "x10_db_ackway": ("the push's last beat clears hit_way's dirty bit, not the pushed way's", [
    (C, "else if (dw_ack) dw_d[p_way] = 1'b0;", "else if (dw_ack) dw_d[hit_way] = 1'b0;")]),
 "x11_rrb_noclr": ("a row clear does not reset the round-robin pointer", [
    (C, ".we1(inv_wren),    .a1(inv_idx),  .d1(2'd0),", ".we1(1'b0),        .a1(inv_idx),  .d1(2'd0),")]),
}

def srcs(rtl):
    src = [os.path.join(rtl, "ap040_pipe_pkg.sv")]
    src += sorted(os.path.join(rtl, x) for x in os.listdir(rtl) if x.startswith("ap040_") and x.endswith(".v"))
    src += sorted(os.path.join(rtl, "compat", x) for x in os.listdir(os.path.join(rtl, "compat")) if x.endswith(".v"))
    return src + [os.path.join(rtl, "compat", "primitives", "dpram.v")]

def build(rtl, d, kind, nochk):
    vvp = os.path.join(d, f"tb_{kind}.vvp")
    rc = os.path.join(rtl, "compat")
    if kind == "snoop":
        cmd = ["iverilog", "-g2012", "-DAP040_PIPE_CACHE", "-DXC7=1", "-I", rc, "-s", "tb_ap040_cache_snoop", "-o", vvp,
               os.path.join(T, "tb_ap040_cache_snoop.v"), os.path.join(rc, "ap040_cache.v"),
               os.path.join(rc, "ap040_xc7_ram2w.v"), os.path.join(rc, "primitives", "dpram.v")]
    else:
        tbs = ["tb_ap040_pipe_compat.v", "tb_sb_check.v", "tb_dfpmis_check.v"] + ([] if nochk else ["tb_xc7_check.v"])
        cmd = ["iverilog", "-g2012"] + DEFS + ["-I", rtl, "-I", rc, "-o", vvp] + [os.path.join(T, t) for t in tbs] + srcs(rtl)
    with open(os.path.join(d, f"clog_{kind}"), "w") as cl:
        if subprocess.run(cmd, stdout=cl, stderr=subprocess.STDOUT).returncode != 0:
            return None
    return vvp

def run(name, out, all_progs, nochk):
    what, edits = MUT[name][0], MUT[name][1]
    equiv = MUT[name][2] if len(MUT[name]) > 2 else ""
    d = os.path.join(out, name)
    shutil.rmtree(d, ignore_errors=True)
    rtl = os.path.join(d, "rtl")
    shutil.copytree(R0, rtl)
    for f, old, new in edits:
        p = os.path.join(rtl, f)
        s = open(p).read()
        if s.count(old) != 1:
            return f"  {name}: EDIT DID NOT APPLY ({f}: {s.count(old)} matches)"
        open(p, "w").write(s.replace(old, new))
    vvps = {}
    res = []
    caught = False
    for prog in PROGS:
        if caught and not all_progs:
            break
        kind = "snoop" if prog == "snoop" else "compat"
        if kind not in vvps:
            vvps[kind] = build(rtl, d, kind, nochk)
            if vvps[kind] is None:
                return f"  {name}: MUTANT DOES NOT COMPILE ({kind})"
        lname = os.path.join(d, f"{prog}.log")
        args = [] if kind == "snoop" else ["+prog=" + os.path.join(out, prog + ".hex")]
        with open(lname, "w") as lg:
            subprocess.run(["timeout", "3600", "vvp", vvps[kind]] + args, stdout=lg, stderr=subprocess.STDOUT)
        log = open(lname).read().splitlines()
        if any("ALL TESTS PASSED" in l for l in log):
            res.append(f"{prog}:pass")
        else:
            why = next((l for l in log if "FAIL" in l), "(no FAIL line: timeout?)")
            res.append(f"{prog}:CAUGHT [{why[:110]}]")
            caught = True
    verdict = "caught" if caught else ("SURVIVED (expected: " + equiv + ")" if equiv else "SURVIVED")
    return f"  {name}: {verdict} -- {what}\n      " + "  ".join(res)

def main():
    a = sys.argv[1:]; j = 3; out = os.path.join(T, "build", "xc7mut"); all_progs = False; nochk = False
    while a and a[0].startswith("-"):
        if a[0] == "-j": j = int(a[1]); a = a[2:]
        elif a[0] == "-o": out = a[1]; a = a[2:]
        elif a[0] == "-a": all_progs = True; a = a[1:]
        elif a[0] == "-n": nochk = True; a = a[1:]
        else: sys.exit("bad option " + a[0])
    names = a or list(MUT)
    os.makedirs(out, exist_ok=True)
    for prog in PROGS:
        if prog == "snoop":
            continue
        subprocess.run(["vasmm68k_mot", "-Fbin", "-m68040", "-no-opt", "-quiet", "-o", os.path.join(out, prog + ".bin"),
                        os.path.join(T, "cache_asm", prog + ".s")], check=True, stderr=subprocess.DEVNULL)
        subprocess.run(["python3", os.path.join(T, "bin2hex.py"), os.path.join(out, prog + ".bin"), os.path.join(out, prog + ".hex")], check=True)
    with ThreadPoolExecutor(j) as ex:
        for r in ex.map(lambda n: run(n, out, all_progs, nochk), names):
            print(r, flush=True)

if __name__ == "__main__":
    main()
