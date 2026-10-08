#!/bin/bash
# fuzz.sh <first-seed> <count> ["defines for the build under test"]:
# differential runs of gen.py's random programs on the compat bench, the
# reference build (every catchup switch off) against the build under test
# (default: all five on).  A seed FAILS when the program or the order
# checker fails, or when the dumps ($3000-$3FFF: the work area and the
# registers) differ.  Runs in parallel (vvp is single-threaded).
set -u
cd "$(dirname "$0")"
T=$(cd ../.. && pwd); R=$T/../rtl; W=${FUZZ_DIR:-$T/build/fuzz}; mkdir -p $W
DEFS=${3:-"-DSTORE_BUF=1 -DFWD=1 -DRAS=1 -DPRECISE=1 -DMISPLIT=1"}
TAG=$(echo "$DEFS" | tr -cd 'A-Z0-9_=' | tr '=' '_')
SRC="$R/ap040_pipe_pkg.sv $(ls $R/ap040_*.v | tr '\n' ' ') $(ls $R/compat/*.v | tr '\n' ' ') $R/compat/primitives/dpram.v"
[ -f $W/ref.vvp ] || iverilog -g2012 -I $R -I $R/compat -o $W/ref.vvp $T/tb_ap040_pipe_compat.v $T/tb_sb_check.v $SRC 2>&1 | grep -v "constant sel"
# FUZZ_TB="<files in tb/>": extra companion benches for the build under test
# only (white-box checkers, e.g. tb_xc7_check.v with -DXC7=1)
XTB=""; for f in ${FUZZ_TB:-}; do XTB="$XTB $T/$f"; done
iverilog -g2012 $DEFS -I $R -I $R/compat -o $W/dut_$TAG.vvp $T/tb_ap040_pipe_compat.v $T/tb_sb_check.v $XTB $SRC 2>&1 | grep -v "constant sel"
one() {
	s=$1
	python3 gen.py $s $W/p$s.s > /dev/null && vasmm68k_mot -Fbin -m68040 -no-opt -quiet -o $W/p$s.bin $W/p$s.s &&
	python3 $T/bin2hex.py $W/p$s.bin $W/p$s.hex || { echo "seed $s: GEN/ASM ERROR"; return; }
	timeout 1800 vvp $W/ref.vvp +prog=$W/p$s.hex +dump=$W/r$s.dmp > $W/r$s.log 2>&1
	timeout 1800 vvp $W/dut_$TAG.vvp +prog=$W/p$s.hex +dump=$W/d$s.dmp > $W/d$s.log 2>&1
	if ! grep -q "ALL TESTS PASSED" $W/r$s.log; then echo "seed $s: reference fails ($(grep -m1 -E 'FAIL' $W/r$s.log | cut -c1-80))"; return; fi
	if ! grep -q "ALL TESTS PASSED" $W/d$s.log; then echo "seed $s: FAIL under test ($(grep -m1 -E 'FAIL' $W/d$s.log | cut -c1-100))"; return; fi
	if ! cmp -s $W/r$s.dmp $W/d$s.dmp; then echo "seed $s: FAIL dump differs ($(diff <(nl $W/r$s.dmp) <(nl $W/d$s.dmp) | head -3 | tr '\n' ' '))"; return; fi
	echo "seed $s: ok"
}
export -f one; export W T TAG
seq $1 $(($1 + $2 - 1)) | xargs -P ${FUZZ_J:-6} -I{} bash -c 'one {}'
