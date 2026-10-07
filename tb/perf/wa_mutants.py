#!/usr/bin/env python3
"""wa_mutants.py [-j N] [-o out-dir] [-a] [names...]: the write-allocate
mutant table (findings/copyback/plan.md S3, AP040_WALLOC).  Each mutant is a
one- or two-line edit of a scratch copy of rtl/, built on the compat bench
with the board switches plus COPYBACK=1 WALLOC=1, the store-order checker
and the white-box checker tb_dfpmis_check.v, and run on cache_asm/
t_cbwa_pipe.s, t_cbsnp_pipe.s and t_cbtab_pipe.s, then t_cbwa_pipe.s and
t_cbwaerr_pipe.s on a FILLCH=0 bench (every fill on the master port, the
board's FILL_CHANNEL = 0; the allocate's fill bus error).  A mutant is CAUGHT when a run does not print ALL TESTS
PASSED; the programs run in that order and stop at the first catch (-a:
run them all).  Runs -j mutants in parallel (vvp is single-threaded;
default 3).  Rows marked "equiv" are argued equivalent (they are expected
to survive; the reason is in the row)."""
import os, subprocess, sys, shutil
from concurrent.futures import ThreadPoolExecutor

T = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))     # tb/
R0 = os.path.join(os.path.dirname(T), "rtl")
DEFS = "-DSTORE_BUF=1 -DFWD=1 -DRAS=1 -DPRECISE=1 -DSB_MMU=1 -DLDX=1 -DBTFN=1 -DCOPYBACK=1 -DDFP_MIS=1 -DWALLOC=1".split()
PROGS = [("t_cbwa_pipe", 1), ("t_cbsnp_pipe", 1), ("t_cbtab_pipe", 1), ("t_cbwa_pipe", 0), ("t_cbwaerr_pipe", 0)]   # (program, FILLCH)
C = "compat/ap040_cache.v"

# name: (what it breaks, [(file, old, new), ...][, "equiv: why"])
MUT = {
 "m1_tagw_snoop": ("the validation merge ignores a snoop landing in the C_TAGW clock (ad7008f class)", [
    (C, "wire wa_ok     = (WALLOC != 0) && (cst == C_TAGW) && wa_v && !fill_snooped && !snoop_fill_row;",
        "wire wa_ok     = (WALLOC != 0) && (cst == C_TAGW) && wa_v && !fill_snooped;")]),
 "m2_fill_snooped": ("the validation merge ignores a snoop during the fill", [
    (C, "wire wa_ok     = (WALLOC != 0) && (cst == C_TAGW) && wa_v && !fill_snooped && !snoop_fill_row;",
        "wire wa_ok     = (WALLOC != 0) && (cst == C_TAGW) && wa_v && !snoop_fill_row;")]),
 "m3_no_drain": ("no write-through fallback when the line is not validated (the store is lost)", [
    (C, "\t\t\t\t\twa_v <= 1'b0;\n\t\t\t\t\tif (!wa_ok) begin", "\t\t\t\t\twa_v <= 1'b0;\n\t\t\t\t\tif (1'b0) begin")]),
 "m4_dirty_always": ("the dirty bit set even when the line is not validated", [
    (C, "db[r_row[5:0]][r_way] <= wa_ok;", "db[r_row[5:0]][r_way] <= ((WALLOC != 0) && wa_v);")]),
 "m5_wrong_word": ("the merge lands in the fill's last beat's word, not the store's", [
    (C, "                   wa_ok  ? {1'b0, r_row[5:0], r_addr[3:2]}", "                   wa_ok  ? {1'b0, r_row[5:0], r_beat}")]),
 "m6_long_merge": ("the merge always longword-sized", [
    (C, "                   wa_ok            ? lw_merge(fill_hold, r_wdata, r_size, r_off) :",
        "                   wa_ok            ? lw_merge(fill_hold, r_wdata, `AP040_SZ_L, r_off) :")]),
 "m7_tagridx": ("the allocate's tag reads at a_row (the core's next request) instead of p_set", [
    (C, "assign tag_ridx  = ((cst == C_PUSH) || (cst == C_WLOOK) || ((WALLOC != 0) && wa_v)) ? {1'b0, p_set} :",
        "assign tag_ridx  = ((cst == C_PUSH) || (cst == C_WLOOK)) ? {1'b0, p_set} :")]),
 "m8_postbusy": ("the walker not held off during the allocate", [
    (C, "assign post_busy = st_posted | dr_active | (cst == C_PUSH) | sweep_push | ((WALLOC != 0) && wa_v);",
        "assign post_busy = st_posted | dr_active | (cst == C_PUSH) | sweep_push;")]),   # FILLCH=0: the bench's walker/bus overlap check
 "m9_rbank": ("r_bank not cleared: the fill's beats go to the bank of the previous fill", [
    (C, "\t\t\t\t\tr_bank    <= 1'b0;\n\t\t\t\t\tp_set     <= r_row[5:0];", "\t\t\t\t\tr_bank    <= r_bank;\n\t\t\t\t\tp_set     <= r_row[5:0];")]),
 "m10_skip_wal": ("no C_WAL launch clock: C_LOOK trusts the st_chk clock's row read (at a_row)", [
    (C, "\t\t\t\t\twa_v      <= 1'b1;\n\t\t\t\t\tcst <= C_WAL;", "\t\t\t\t\twa_v      <= 1'b1;\n\t\t\t\t\tcst <= C_LOOK;")],
    "equiv by protocol: the core holds the store's request through the clock after the ack, so a_row is the store's row there"),
 "m11_hitguard": ("C_LOOK's hit branch not masked by wa_v", [
    (C, "\t\t\t\t    !((WALLOC != 0) && wa_v)) begin", "\t\t\t\t    1'b1) begin")],
    "equiv: the line is absent (the verdict was a miss) and nothing can fill it before C_LOOK"),
 "m12_ack_tagw": ("ack_r raised in the allocate's C_TAGW (a spurious ack to the core's next request)", [
    (C, "\t\t\t\t\twa_v <= 1'b0;\n\t\t\t\t\tif (!wa_ok) begin", "\t\t\t\t\twa_v <= 1'b0; ack_r <= 1;\n\t\t\t\t\tif (!wa_ok) begin")]),
 "m13_merge_hitway": ("the merge written into hit_way instead of the filled way", [
    (C, "                   wa_ok                                   ? (4'd1 << r_way) :",
        "                   wa_ok                                   ? (4'd1 << hit_way) :")]),
 "m14_go_snooped": ("allocate even when a snoop reached the store's window", [
    (C, "                 !mis_v && !look_hit && !st_snooped && !snoop_st_row_pass && !m_err;",
        "                 !mis_v && !look_hit && !m_err;")],
    "equiv in simulation: a snooped window's compare is untrustworthy on silicon only (5.2c); keep the guard"),
 "m15_go_mis": ("two-piece stores on the one-piece allocate path", [
    (C, "                 !mis_v && !look_hit && !st_snooped && !snoop_st_row_pass && !m_err;",
        "                 !look_hit && !st_snooped && !snoop_st_row_pass && !m_err;")],
    "equiv: the C_MST2 branch precedes wa_go in C_PASS's if-chain; !mis_v documents it"),
 "m16_instr": ("the allocate's master-port fill goes out with the next request's c_instr", [
    (C, "assign m_instr = (dr_active | st_posted | push_out | wa_m) ? 1'b0 : c_instr;",
        "assign m_instr = (dr_active | st_posted | push_out) ? 1'b0 : c_instr;")]),
 "m17_fc": ("the allocate's master-port fill goes out with the next request's c_fc", [
    (C, "((st_posted | wa_m) ? r_fc : c_fc);", "(st_posted ? r_fc : c_fc);")]),
 "m18_go_cb": ("a write-through store miss allocates too (r_cb dropped from wa_go)", [
    (C, "wire wa_go     = (WALLOC != 0) && (cst == C_PASS) && st_chk && st_posted && r_cb &&",
        "wire wa_go     = (WALLOC != 0) && (cst == C_PASS) && st_chk && st_posted &&")]),
 "m19_ferr": ("a fill error during the allocate not reported (no post_err)", [
    (C, "if ((WALLOC != 0) && wa_v) begin post_err <= 1'b1; wa_v <= 1'b0; end   // (WALLOC) as C_FILLC",
        "if ((WALLOC != 0) && wa_v) begin wa_v <= 1'b0; end   // (WALLOC) as C_FILLC")]),
 "m20_ferr_hold": ("a fill error during the allocate sets err_hold (the core's next request is wedged)", [
    (C, "if ((WALLOC != 0) && wa_v) begin post_err <= 1'b1; wa_v <= 1'b0; end   // (WALLOC) as C_FILLC\n\t\t\t\t\telse err_hold <= 1;",
        "if ((WALLOC != 0) && wa_v) begin post_err <= 1'b1; wa_v <= 1'b0; end   // (WALLOC) as C_FILLC\n\t\t\t\t\terr_hold <= 1;")],
    "equiv on this bench: post_err halts the core in the same clock, a wedged next request is not observable"),
}

def build(name, rtl, d, fillch):
    src = [os.path.join(rtl, "ap040_pipe_pkg.sv")]
    src += sorted(os.path.join(rtl, x) for x in os.listdir(rtl) if x.startswith("ap040_") and x.endswith(".v"))
    src += sorted(os.path.join(rtl, "compat", x) for x in os.listdir(os.path.join(rtl, "compat")) if x.endswith(".v"))
    src += [os.path.join(rtl, "compat", "primitives", "dpram.v")]
    vvp = os.path.join(d, f"tb_f{fillch}.vvp")
    cmd = ["iverilog", "-g2012"] + DEFS + [f"-Ptb_ap040_pipe_compat.FILLCH={fillch}", "-I", rtl, "-I", os.path.join(rtl, "compat"), "-o", vvp,
           os.path.join(T, "tb_ap040_pipe_compat.v"), os.path.join(T, "tb_sb_check.v"), os.path.join(T, "tb_dfpmis_check.v")] + src
    with open(os.path.join(d, f"clog_f{fillch}"), "w") as cl:
        if subprocess.run(cmd, stdout=cl, stderr=subprocess.STDOUT).returncode != 0:
            return None
    return vvp

def run(name, out, all_progs):
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
    for prog, fillch in PROGS:
        if caught and not all_progs:
            break
        if fillch not in vvps:
            vvps[fillch] = build(name, rtl, d, fillch)
            if vvps[fillch] is None:
                return f"  {name}: MUTANT DOES NOT COMPILE"
        lname = os.path.join(d, f"{prog}_f{fillch}.log")
        with open(lname, "w") as lg:
            subprocess.run(["timeout", "3600", "vvp", vvps[fillch], "+prog=" + os.path.join(out, prog + ".hex")],
                           stdout=lg, stderr=subprocess.STDOUT)
        log = open(lname).read().splitlines()
        if any("ALL TESTS PASSED" in l for l in log):
            res.append(f"{prog}/f{fillch}:pass")
        else:
            why = next((l for l in log if "FAIL" in l), "(no FAIL line: timeout?)")
            res.append(f"{prog}/f{fillch}:CAUGHT [{why[:110]}]")
            caught = True
    verdict = "caught" if caught else ("SURVIVED (expected: " + equiv + ")" if equiv else "SURVIVED")
    return f"  {name}: {verdict} -- {what}\n      " + "  ".join(res)

def main():
    a = sys.argv[1:]; j = 3; out = os.path.join(T, "build", "wamut"); all_progs = False
    while a and a[0].startswith("-"):
        if a[0] == "-j": j = int(a[1]); a = a[2:]
        elif a[0] == "-o": out = a[1]; a = a[2:]
        elif a[0] == "-a": all_progs = True; a = a[1:]
        else: sys.exit("bad option " + a[0])
    names = a or list(MUT)
    os.makedirs(out, exist_ok=True)
    for prog in sorted(set(p for p, _ in PROGS)):
        subprocess.run(["vasmm68k_mot", "-Fbin", "-m68040", "-no-opt", "-quiet", "-o", os.path.join(out, prog + ".bin"),
                        os.path.join(T, "cache_asm", prog + ".s")], check=True, stderr=subprocess.DEVNULL)
        subprocess.run(["python3", os.path.join(T, "bin2hex.py"), os.path.join(out, prog + ".bin"), os.path.join(out, prog + ".hex")], check=True)
    with ThreadPoolExecutor(j) as ex:
        for r in ex.map(lambda n: run(n, out, all_progs), names):
            print(r, flush=True)

if __name__ == "__main__":
    main()
