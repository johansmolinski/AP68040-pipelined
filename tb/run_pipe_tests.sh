#!/bin/sh
# AP040_PIPE self-test suite -- the pipelined core (rtl/ap040_pipe_core.v and
# its stage files).  Needs only iverilog.  Most benches poke their program
# straight into the L1 array (dut.u_l1.mem[...]) at time 0.
#
# The source list is rtl/ap040_pipe_pkg.sv (a package: compiled first) plus
# every rtl/ap040_*.v -- rtl_old/ is never needed.
#
# Minimig plan M1: the milestone-12 unit bench tb_ap040_pipe_l1_wbuf.v is
# retired with the one-entry write buffer it tested (ap040_pipe_l1.v was
# rewritten with byte-granular read/write ports; tb_ap040_pipe_l1_bytes.v
# tests those).
set -eu
cd "$(dirname "$0")"

RTL=../rtl
WORK=build
mkdir -p "$WORK"

SRC="$RTL/ap040_pipe_pkg.sv $(ls $RTL/ap040_*.v | tr '\n' ' ')"
# PIPE_STORE_BUF=<n>: build the program and wrapper benches with that store
# buffer mode (findings/storebuf/plan.md: 0 off, 1 posting, 2 the A/B
# reference); the bus-mode benches check the store order with tb_sb_check.v
SBDEF="${PIPE_STORE_BUF:+-DSTORE_BUF=$PIPE_STORE_BUF} ${PIPE_FWD:+-DFWD=$PIPE_FWD} ${PIPE_RAS:+-DRAS=$PIPE_RAS} ${PIPE_PRECISE:+-DPRECISE=$PIPE_PRECISE} ${PIPE_MISPLIT:+-DMISPLIT=$PIPE_MISPLIT} ${PIPE_SB_MMU:+-DSB_MMU=$PIPE_SB_MMU} ${PIPE_COPYBACK:+-DCOPYBACK=$PIPE_COPYBACK} ${PIPE_BTB:+-DBTB=$PIPE_BTB} ${PIPE_LDX:+-DLDX=$PIPE_LDX} ${PIPE_BTFN:+-DBTFN=$PIPE_BTFN} ${PIPE_DFP_MIS:+-DDFP_MIS=$PIPE_DFP_MIS} ${PIPE_WALLOC:+-DWALLOC=$PIPE_WALLOC}"
# PIPE_SB_MMU=1: stores posted with translation on too (store buffer stage 4)
# PIPE_LDX=1: late-operand load dispatch (findings/loadstore/plan.md step 1)
# PIPE_BTFN=1: forward conditional branches guessed not taken (plan section 11)
# PIPE_DFP_MIS=1: misaligned reads served by the data read path (findings/loadstore/plan.md step 3)
# PIPE_WALLOC=1: write-allocate on a copyback store miss (findings/copyback/plan.md S3; needs PIPE_COPYBACK=1)
# PIPE_PRECISE=1 / PIPE_MISPLIT=1: address-precise fast reads, misaligned transfers split (catchup)
# PIPE_RAS=1: ID's return-address stack (findings/catchup/plan.md step 2), every bench
# PIPE_FWD=1: the wrapper benches with store-to-load forwarding (findings/catchup/plan.md)

# milestone benches (1-17)
TESTS="nop moveq add bra bcc bccw bccl scc dbcc move_mem move_disp jmp bsr jsr exc sup rts_rte addrerr"
# Minimig plan benches, by milestone
TESTS="$TESTS red_movew red_rte_fmt2 red_vbr red_aline_fline"               # M0 red legs, green since M1
TESTS="$TESTS ${PIPE_EXTRA_TESTS:-}"

echo "== compiling pipe benches =="
for t in $TESTS; do
	iverilog -g2012 -I "$RTL" -o "$WORK/tb_pipe_$t.vvp" "tb_ap040_pipe_$t.v" $SRC > "$WORK/tb_pipe_$t.clog" 2>&1 || {
		echo "  COMPILE-ERROR $t"; grep -v "sorry: constant selects" "$WORK/tb_pipe_$t.clog" | head -5; exit 1; }
done

echo "== running =="
fail=0
for t in $TESTS; do
	if timeout 600 vvp "$WORK/tb_pipe_$t.vvp" > "$WORK/pipe_$t.log" 2>&1 && grep -q "ALL TESTS PASSED" "$WORK/pipe_$t.log"; then
		echo "  pass  $t"
	else
		echo "  FAIL  $t  (see $WORK/pipe_$t.log)"
		fail=1
	fi
done

# program benches: pipe_asm/<name>.s + <name>.exp through tb_ap040_pipe_prog.v
# (needs vasmm68k_mot; the .exp files were made from a reference-core run
# with findings/ap040-pipelined/tests/mkprog.sh and reviewed)
if command -v vasmm68k_mot > /dev/null; then
	iverilog -g2012 $SBDEF -I "$RTL" -o "$WORK/tb_pipe_prog.vvp" tb_ap040_pipe_prog.v $SRC > "$WORK/tb_pipe_prog.clog" 2>&1 || {
		echo "  COMPILE-ERROR prog bench"; grep -v "constant selects" "$WORK/tb_pipe_prog.clog" | head -5; exit 1; }
	for s in pipe_asm/*.s; do
		n=$(basename "$s" .s)
		[ -f "pipe_asm/$n.exp" ] || continue
		# the FPU programs need HAS_FPU = 1 and the stub unit: their own leg below
		case "$n" in fpu*) continue ;; esac
		# bus errors exist only on the memory port: such programs run in the bus legs only
		if grep -q "expect-berr" "$s"; then
			( cd pipe_asm && vasmm68k_mot -Fbin -m68040 -no-opt -quiet -o "../$WORK/$n.bin" "$n.s" ) && python3 bin2hex.py "$WORK/$n.bin" "$WORK/$n.hex"
			continue
		fi
		( cd pipe_asm && vasmm68k_mot -Fbin -m68040 -no-opt -quiet -o "../$WORK/$n.bin" "$n.s" ) || { echo "  FAIL  prog:$n (assembler)"; fail=1; continue; }
		python3 bin2hex.py "$WORK/$n.bin" "$WORK/$n.hex"
		# a program's "; diff: --cycles N" line sets its clock budget here too
		cyc=$(sed -n 's/^; diff:.*--cycles \([0-9]*\).*/\1/p' "$s" | head -1)
		if timeout 600 vvp "$WORK/tb_pipe_prog.vvp" +prog="$WORK/$n.hex" +expect="pipe_asm/$n.exp" +cycles=${cyc:-20000} > "$WORK/prog_$n.log" 2>&1 &&
		   grep -q "ALL TESTS PASSED" "$WORK/prog_$n.log"; then
			echo "  pass  prog:$n"
		else
			echo "  FAIL  prog:$n  (see $WORK/prog_$n.log)"
			fail=1
		fi
	done
	# the same programs through the memory port (BUS=1, plan M5) at the three
	# wait profiles of tb_ap040_pipe_prog.v's memory model
	if [ -z "${PIPE_NO_BUS:-}" ]; then
	iverilog -g2012 $SBDEF -DBUS_MODE -I "$RTL" -o "$WORK/tb_pipe_bus.vvp" tb_ap040_pipe_prog.v tb_sb_check.v $SRC > "$WORK/tb_pipe_bus.clog" 2>&1 || {
		echo "  COMPILE-ERROR bus bench"; grep -v "constant selects" "$WORK/tb_pipe_bus.clog" | head -5; exit 1; }
	for s in pipe_asm/*.s; do
		n=$(basename "$s" .s)
		[ -f "pipe_asm/$n.exp" ] || continue
		case "$n" in fpu*) continue ;; esac
		[ -f "$WORK/$n.hex" ] || continue
		# a late (posted-store) bus error exists only with posting on
		if grep -q "^lberr" "pipe_asm/$n.exp" && [ "${PIPE_STORE_BUF:-0}" != 1 ]; then continue; fi
		cyc=$(sed -n 's/^; diff:.*--cycles \([0-9]*\).*/\1/p' "$s" | head -1)
		cyc=$(( ${cyc:-20000} * 8 ))
		for p in 0 1 2; do
			if timeout 900 vvp "$WORK/tb_pipe_bus.vvp" +prog="$WORK/$n.hex" +expect="pipe_asm/$n.exp" +prof=$p +cycles=$cyc > "$WORK/bus_${n}_$p.log" 2>&1 &&
			   grep -q "ALL TESTS PASSED" "$WORK/bus_${n}_$p.log"; then
				echo "  pass  bus$p:$n"
			else
				echo "  FAIL  bus$p:$n  (see $WORK/bus_${n}_$p.log)"
				fail=1
			fi
		done
	done
	fi

	# plan M10.1: the core built with HAS_FPU = 1, with tb_fpu_stub.v -- a
	# fixed-latency stand-in for ap040_fpu -- on its fp_* port group, so the
	# req/accepted/done interlock is testable before the real unit goes in.
	# The pipe_asm/fpu*.s programs run only here; with HAS_FPU = 0 every one
	# of their opcodes is M10.0's F-line exception instead (fline4.s).
	iverilog -g2012 $SBDEF -DFPU_STUB -I "$RTL" -o "$WORK/tb_pipe_fpu.vvp" tb_ap040_pipe_prog.v tb_fpu_stub.v $SRC > "$WORK/tb_pipe_fpu.clog" 2>&1 || {
		echo "  COMPILE-ERROR fpu prog bench"; grep -v "constant selects" "$WORK/tb_pipe_fpu.clog" | head -5; exit 1; }
	iverilog -g2012 $SBDEF -DFPU_STUB -DBUS_MODE -I "$RTL" -o "$WORK/tb_fpu_bus.vvp" tb_ap040_pipe_prog.v tb_fpu_stub.v tb_sb_check.v $SRC > "$WORK/tb_fpu_bus.clog" 2>&1 || {
		echo "  COMPILE-ERROR fpu bus bench"; grep -v "constant selects" "$WORK/tb_fpu_bus.clog" | head -5; exit 1; }
	for s in pipe_asm/fpu*.s; do
		[ -f "$s" ] || continue
		n=$(basename "$s" .s)
		[ -f "pipe_asm/$n.exp" ] || continue
		# fpureal.s answers to the REAL unit, not the stub: its own leg below
		case "$n" in fpureal*) continue ;; esac
		( cd pipe_asm && vasmm68k_mot -Fbin -m68040 -no-opt -quiet -o "../$WORK/$n.bin" "$n.s" ) || { echo "  FAIL  fpu:$n (assembler)"; fail=1; continue; }
		python3 bin2hex.py "$WORK/$n.bin" "$WORK/$n.hex"
		cyc=$(sed -n 's/^; diff:.*--cycles \([0-9]*\).*/\1/p' "$s" | head -1)
		if timeout 600 vvp "$WORK/tb_pipe_fpu.vvp" +prog="$WORK/$n.hex" +expect="pipe_asm/$n.exp" +cycles=${cyc:-20000} > "$WORK/fpu_$n.log" 2>&1 &&
		   grep -q "ALL TESTS PASSED" "$WORK/fpu_$n.log"; then
			echo "  pass  fpu:$n"
		else
			echo "  FAIL  fpu:$n  (see $WORK/fpu_$n.log)"
			fail=1
		fi
		if [ -z "${PIPE_NO_BUS:-}" ]; then
			for p in 0 1 2; do
				if timeout 900 vvp "$WORK/tb_fpu_bus.vvp" +prog="$WORK/$n.hex" +expect="pipe_asm/$n.exp" +prof=$p +cycles=$(( ${cyc:-20000} * 8 )) > "$WORK/fpubus_${n}_$p.log" 2>&1 &&
				   grep -q "ALL TESTS PASSED" "$WORK/fpubus_${n}_$p.log"; then
					echo "  pass  fpubus$p:$n"
				else
					echo "  FAIL  fpubus$p:$n  (see $WORK/fpubus_${n}_$p.log)"
					fail=1
				fi
			done
		fi
	done
	# plan M10.1 step 3: the REAL ap040_fpu (rtl/compat/) on the same port
	# group, instantiated by the same include the wrapper uses, so these
	# programs drive the instance the bitstream carries.  pipe_asm/fpureal*.s
	# holds hand-computed IEEE results, which the stub could not produce.
	FSRC="$SRC $RTL/compat/ap040_fpu.v"
	iverilog -g2012 $SBDEF -DFPU_REAL -I "$RTL" -I "$RTL/compat" -o "$WORK/tb_pipe_fpr.vvp" tb_ap040_pipe_prog.v $FSRC > "$WORK/tb_pipe_fpr.clog" 2>&1 || {
		echo "  COMPILE-ERROR fpu-real prog bench"; grep -v "constant selects" "$WORK/tb_pipe_fpr.clog" | head -5; exit 1; }
	iverilog -g2012 $SBDEF -DFPU_REAL -DBUS_MODE -I "$RTL" -I "$RTL/compat" -o "$WORK/tb_fpr_bus.vvp" tb_ap040_pipe_prog.v tb_sb_check.v $FSRC > "$WORK/tb_fpr_bus.clog" 2>&1 || {
		echo "  COMPILE-ERROR fpu-real bus bench"; grep -v "constant selects" "$WORK/tb_fpr_bus.clog" | head -5; exit 1; }
	for s in pipe_asm/fpureal*.s; do
		[ -f "$s" ] || continue
		n=$(basename "$s" .s)
		[ -f "pipe_asm/$n.exp" ] || continue
		( cd pipe_asm && vasmm68k_mot -Fbin -m68040 -no-opt -quiet -o "../$WORK/$n.bin" "$n.s" ) || { echo "  FAIL  fpr:$n (assembler)"; fail=1; continue; }
		python3 bin2hex.py "$WORK/$n.bin" "$WORK/$n.hex"
		cyc=$(sed -n 's/^; diff:.*--cycles \([0-9]*\).*/\1/p' "$s" | head -1)
		# bus errors exist only on the memory port: such programs run in the bus legs only
		if grep -q "expect-berr" "$s"; then :
		elif timeout 900 vvp "$WORK/tb_pipe_fpr.vvp" +prog="$WORK/$n.hex" +expect="pipe_asm/$n.exp" +cycles=${cyc:-20000} > "$WORK/fpr_$n.log" 2>&1 &&
		   grep -q "ALL TESTS PASSED" "$WORK/fpr_$n.log"; then
			echo "  pass  fpr:$n"
		else
			echo "  FAIL  fpr:$n  (see $WORK/fpr_$n.log)"
			fail=1
		fi
		if [ -z "${PIPE_NO_BUS:-}" ]; then
			for p in 0 1 2; do
				if timeout 1800 vvp "$WORK/tb_fpr_bus.vvp" +prog="$WORK/$n.hex" +expect="pipe_asm/$n.exp" +prof=$p +cycles=$(( ${cyc:-20000} * 8 )) > "$WORK/fprbus_${n}_$p.log" 2>&1 &&
				   grep -q "ALL TESTS PASSED" "$WORK/fprbus_${n}_$p.log"; then
					echo "  pass  fprbus$p:$n"
				else
					echo "  FAIL  fprbus$p:$n  (see $WORK/fprbus_${n}_$p.log)"
					fail=1
				fi
			done
		fi
	done
	# the FPSP (2026-09-24, plan M10): Motorola's M68040 FPSP exactly as
	# Thomas Richter's 68040.library 40.2 carries it, installed by the
	# library's own code, runs every floating-point instruction the 68040
	# does not implement (tb/mk_fpsp.py wrote the program and its expected
	# values).  The library is read by path and never copied into this tree
	# (FPSP_LIB); without it the leg says so and is skipped.  256 KB of
	# memory (L1_AW=17); the L1 leg and the three bus profiles run in
	# parallel (vvp is single threaded).
	FPSP_LIB=${FPSP_LIB:-/home/paul/work/amiga/adc/Contributions/Thomas_Richter/Tools/Debugging/MMULib/Libs/68040.library}
	if [ -f "$FPSP_LIB" ] && python3 mk_fpsplib.py "$FPSP_LIB" "$WORK/fpsp040lib.bin" "$WORK/fpsp040lib.inc" > "$WORK/fpsplib.log" 2>&1; then
		( cd fpsp_asm && vasmm68k_mot -Fbin -m68040 -m68882 -no-opt -quiet -o "../$WORK/fpsp_lib.bin" fpsp_lib.s ) &&
		python3 bin2hex.py "$WORK/fpsp_lib.bin" "$WORK/fpsp_lib.hex" || { echo "  FAIL  fpsp:fpsp_lib (assembler)"; fail=1; }
		iverilog -g2012 $SBDEF -DFPU_REAL -Ptb_ap040_pipe_prog.L1_AW=17 -I "$RTL" -I "$RTL/compat" -o "$WORK/tb_fpsp.vvp" tb_ap040_pipe_prog.v $FSRC > "$WORK/tb_fpsp.clog" 2>&1 &&
		iverilog -g2012 $SBDEF -DFPU_REAL -DBUS_MODE -Ptb_ap040_pipe_prog.L1_AW=17 -I "$RTL" -I "$RTL/compat" -o "$WORK/tb_fpsp_bus.vvp" tb_ap040_pipe_prog.v tb_sb_check.v $FSRC > "$WORK/tb_fpsp_bus.clog" 2>&1 || {
			echo "  COMPILE-ERROR fpsp bench"; grep -v "constant selects" "$WORK/tb_fpsp.clog" "$WORK/tb_fpsp_bus.clog" | head -5; exit 1; }
		timeout 1800 vvp "$WORK/tb_fpsp.vvp" +prog="$WORK/fpsp_lib.hex" +expect=fpsp_asm/fpsp_lib.exp +cycles=4000000 > "$WORK/fpsp_lib.log" 2>&1 &
		for p in 0 1 2; do
			timeout 1800 vvp "$WORK/tb_fpsp_bus.vvp" +prog="$WORK/fpsp_lib.hex" +expect=fpsp_asm/fpsp_lib.exp +prof=$p +cycles=32000000 > "$WORK/fpspbus_$p.log" 2>&1 &
		done
		wait
		for l in fpsp_lib fpspbus_0 fpspbus_1 fpspbus_2; do
			if grep -q "ALL TESTS PASSED" "$WORK/$l.log"; then echo "  pass  fpsp:$l"; else echo "  FAIL  fpsp:$l  (see $WORK/$l.log)"; fail=1; fi
		done
	else
		echo "  (68040.library not found at $FPSP_LIB, or not version 40.2: FPSP legs skipped)"
	fi
	# the background-release bench (plan M10.1(c)): the one property a program
	# cannot see, because it is invisible to the architecture
	for t in fpu_bg; do
		iverilog -g2012 -I "$RTL" -o "$WORK/tb_pipe_$t.vvp" "tb_ap040_pipe_$t.v" tb_fpu_stub.v $SRC > "$WORK/tb_pipe_$t.clog" 2>&1 || {
			echo "  COMPILE-ERROR $t"; grep -v "sorry: constant selects" "$WORK/tb_pipe_$t.clog" | head -5; exit 1; }
		if timeout 600 vvp "$WORK/tb_pipe_$t.vvp" > "$WORK/pipe_$t.log" 2>&1 && grep -q "ALL TESTS PASSED" "$WORK/pipe_$t.log"; then
			echo "  pass  $t"
		else
			echo "  FAIL  $t  (see $WORK/pipe_$t.log)"
			fail=1
		fi
	done
else
	echo "  (vasmm68k_mot not found: program benches skipped)"
fi

# the wrapper (plan M5): ap040_pipe_tg68k_compat with the reference's cache,
# MMU (M7) and 16-bit adapter, lib/AP68040's program bench (three wait
# profiles) on the reference's t_integer.s, t_bitfield_mmu.s and t_mmu.s.  AP040_REF points at lib/AP68040.
# (2026-10-05) lib/AP68040 next to this submodule (../../AP68040 from tb/)
# first, then the old sibling-clone layout.  `|| true`: under set -eu a
# failed command substitution in this assignment ended the script silently
# right here -- no compat legs, no final line, yet no FAIL either.
AP040_REF=${AP040_REF:-$( { cd ../../AP68040 2>/dev/null || cd ../../MinimigAGA_TC64/lib/AP68040 2>/dev/null; } && pwd || true)}
if [ -n "$AP040_REF" ] && [ -f "$AP040_REF/tb/asm/t_integer.s" ] && command -v vasmm68k_mot > /dev/null; then
	CSRC="$SRC $(ls $RTL/compat/*.v | tr '\n' ' ') $RTL/compat/primitives/dpram.v"
	# (PIPE_DFP_MIS) the misaligned read path's white-box collision checker joins the bench
	iverilog -g2012 $SBDEF -I "$RTL" -I "$RTL/compat" -o "$WORK/tb_compat.vvp" tb_ap040_pipe_compat.v tb_sb_check.v ${PIPE_DFP_MIS:+tb_dfpmis_check.v} $CSRC > "$WORK/tb_compat.clog" 2>&1 || {
		echo "  COMPILE-ERROR compat bench"; grep -v "constant selects" "$WORK/tb_compat.clog" | head -5; exit 1; }
	# lib/AP68040's reset bench on the wrapper, and its adapter unit benches
	# on the lifted copies
	iverilog -g2012 -I "$RTL" -I "$RTL/compat" -o "$WORK/tb_preset.vvp" tb_ap040_pipe_reset.v $CSRC > "$WORK/tb_preset.clog" 2>&1 &&
	timeout 600 vvp "$WORK/tb_preset.vvp" > "$WORK/compat_reset.log" 2>&1 && grep -q "ALL TESTS PASSED" "$WORK/compat_reset.log" &&
		echo "  pass  compat:reset" || { echo "  FAIL  compat:reset  (see $WORK/compat_reset.log)"; fail=1; }
	for u in bus16_gap bus_timeout; do
		iverilog -g2012 -I "$RTL/compat" -o "$WORK/tb_$u.vvp" tb_ap040_$u.v "$RTL/compat/ap040_bus16_adapter.v" "$RTL/compat/ap040_bus_timeout.v" > "$WORK/tb_$u.clog" 2>&1 &&
		vvp "$WORK/tb_$u.vvp" > "$WORK/unit_$u.log" 2>&1 && grep -q "ALL TESTS PASSED" "$WORK/unit_$u.log" &&
			echo "  pass  unit:$u" || { echo "  FAIL  unit:$u  (see $WORK/unit_$u.log)"; fail=1; }
	done
	# t_mmu.s through mk_tmmu.py (the manual's WB1 for a faulted write, PLAN
	# D13; no interrupts or STOP before M9)
	python3 mk_tmmu.py "$AP040_REF/tb/asm/t_mmu.s" m9s > "$WORK/t_mmu_m9s.s"
	# t_exceptions.s through mk_tint.py: every section up to the M9 subset
	# (interrupts, STOP, the level-sensitive IPL) PLUS the trace section,
	# which M9.T turned on -- T1, T0, the synchronisation list, the traced
	# STOP and the MOVEC direction rule.  The M bit section joins it when
	# M9.T's last step lands.
	python3 mk_tint.py "$AP040_REF/tb/asm/t_exceptions.s" exc_m9t > "$WORK/texc_m9t.s"
	# apolkosnik/AP68040 main's t_movem_restart.s (MOVEM CM continuation, PLAN
	# D15) when that clone is next to this one (case 7: an interrupt behind the CM restart)
	TMR=""
	AP040_MAIN=${AP040_MAIN:-../../AP68040-reference}
	if [ -f "$AP040_MAIN/tb/asm/t_movem_restart.s" ]; then
		python3 mk_tmovem.py "$AP040_MAIN/tb/asm/t_movem_restart.s" m9s > "$WORK/t_movem_restart_m9s.s" && TMR=t_movem_restart_m9s
		# (2026-09-24) the upstream state-frame programs, revision $41: the
		# BUSY frame's CU_SAVEPC=$FE resume (57 cases) and the frame
		# lifecycle -- sizes, pointers, format errors, the deferred E1
		# frame an FSAVE extracts and an FRESTORE re-arms
		[ -f "$AP040_MAIN/tb/asm/t_fpu_resume.s" ] && TMR="$TMR t_fpu_resume t_fpu_frames"
	else
		echo "  (AP68040-reference not found: t_movem_restart skipped)"
	fi
	# t_irqwedge_pipe (2026-09-23, the board's black screen): an interrupt
	# landing on an instruction past P_START with no read in flight.  The
	# failure is a WEDGE, so it gets a short phase timeout (it passes in
	# 180k clocks) instead of the bench's 20M default.
	for t in t_integer t_cache t_bitfield_mmu t_mmu_m9s t_mmu_pipe t_rmw_wp texc_m9t t_irq_pipe t_mbit_pipe t_trirq_pipe t_ipend_pipe t_irqwedge_pipe t_irqlat_pipe t_icache_pipe t_earlydrop_pipe t_specread_pipe t_dcache_pipe t_sbuf_pipe t_fwd_pipe t_sbmmu_pipe t_cb_pipe t_cbtab_pipe t_cbsnp_pipe t_specexc_pipe t_dfpmis_pipe t_cbwa_pipe $TMR; do
		if [ "$t" = t_mmu_m9s ] || [ "$t" = t_movem_restart_m9s ] || [ "$t" = texc_m9t ]; then
			vasmm68k_mot -Fbin -m68040 -no-opt -quiet -o "$WORK/$t.bin" "$WORK/$t.s"
		elif [ "$t" = t_mmu_pipe ] || [ "$t" = t_rmw_wp ]; then
			vasmm68k_mot -Fbin -m68040 -no-opt -quiet -o "$WORK/$t.bin" "mmu_asm/$t.s"
		elif [ "$t" = t_icache_pipe ] || [ "$t" = t_earlydrop_pipe ] || [ "$t" = t_specread_pipe ] || [ "$t" = t_dcache_pipe ] || [ "$t" = t_sbuf_pipe ] || [ "$t" = t_fwd_pipe ] || [ "$t" = t_sbmmu_pipe ] || [ "$t" = t_cb_pipe ] || [ "$t" = t_cbtab_pipe ] || [ "$t" = t_cbsnp_pipe ] || [ "$t" = t_specexc_pipe ] || [ "$t" = t_dfpmis_pipe ] || [ "$t" = t_cbwa_pipe ]; then
			# plan M14: the pipelined instruction read path's coherency rules
			vasmm68k_mot -Fbin -m68040 -no-opt -quiet -o "$WORK/$t.bin" "cache_asm/$t.s"
		elif [ "$t" = t_irq_pipe ] || [ "$t" = t_mbit_pipe ] || [ "$t" = t_trirq_pipe ] || [ "$t" = t_ipend_pipe ] || [ "$t" = t_irqwedge_pipe ] || [ "$t" = t_irqlat_pipe ]; then
			vasmm68k_mot -Fbin -m68040 -no-opt -quiet -o "$WORK/$t.bin" "irq_asm/$t.s"
		elif [ "$t" = t_fpu_resume ] || [ "$t" = t_fpu_frames ]; then
			( cd "$AP040_MAIN/tb/asm" && vasmm68k_mot -Fbin -m68040 -no-opt -quiet -o "$OLDPWD/$WORK/$t.bin" "$t.s" )
		else
			( cd "$AP040_REF/tb/asm" && vasmm68k_mot -Fbin -m68040 -no-opt -quiet -o "$OLDPWD/$WORK/$t.bin" "$t.s" )
		fi
		python3 bin2hex.py "$WORK/$t.bin" "$WORK/$t.hex"
		tmo=""; [ "$t" = t_irqwedge_pipe ] && tmo="+timeout=2000000"
		if timeout 1800 vvp "$WORK/tb_compat.vvp" +prog="$WORK/$t.hex" $tmo > "$WORK/compat_$t.log" 2>&1 &&
		   grep -q "ALL TESTS PASSED" "$WORK/compat_$t.log"; then
			echo "  pass  compat:$t"
		else
			echo "  FAIL  compat:$t  (see $WORK/compat_$t.log)"
			fail=1
		fi
	done
	# (audit 2026-09-27) a +prog= path longer than 128 characters: the bench
	# kept it in a reg [1023:0] and cut it off, so the program never loaded
	LP="$WORK/longpath_$(printf 'x%.0s' $(seq 1 140))"
	mkdir -p "$LP" && cp "$WORK/t_rmw_wp.hex" "$LP/p.hex" &&
	timeout 1800 vvp "$WORK/tb_compat.vvp" +prog="$LP/p.hex" > "$WORK/compat_longpath.log" 2>&1 &&
	grep -q "ALL TESTS PASSED" "$WORK/compat_longpath.log" &&
		echo "  pass  compat:longpath" || { echo "  FAIL  compat:longpath  (see $WORK/compat_longpath.log)"; fail=1; }
	# the 2^31-word freeze: IF's word count preset 256 short of the wrapper's
	# PROG_WORDS part way into t_integer (tb_issued_wrap.v); it must not wedge
	iverilog -g2012 $SBDEF -DISSUED_T=200000 -I "$RTL" -I "$RTL/compat" -o "$WORK/tb_issued_wrap.vvp" \
		tb_ap040_pipe_compat.v tb_issued_wrap.v tb_sb_check.v $CSRC > "$WORK/tb_issued_wrap.clog" 2>&1 &&
	timeout 1800 vvp "$WORK/tb_issued_wrap.vvp" +prog="$WORK/t_integer.hex" +timeout=2000000 > "$WORK/compat_issued_wrap.log" 2>&1 &&
	grep -q "ALL TESTS PASSED" "$WORK/compat_issued_wrap.log" && grep -q "issued_wrap: IF's word count preset" "$WORK/compat_issued_wrap.log" &&
		echo "  pass  compat:issued_wrap" || { echo "  FAIL  compat:issued_wrap  (see $WORK/compat_issued_wrap.log)"; fail=1; }
else
	echo "  (lib/AP68040 not found: compat bench skipped)"
fi

if [ $fail -eq 0 ]; then echo "AP040_PIPE: ALL TESTS PASSED"; else echo "AP040_PIPE: FAILURES"; exit 1; fi
