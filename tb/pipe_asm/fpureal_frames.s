; fpureal_frames.s - FSAVE/FRESTORE stress: every place the unit prepares a
; state frame, each saved, the registers corrupted, restored and saved again.
;
; The unit writes its frame payload (ap040_fpu.v fstate_*) from many places:
; the dispatch cycle (register operands, packed and FMOVECR), the source
; conversion (memory operands: single/double/extended denormals and
; unnormals), the destination checks in F_EXEC (FCMP) and F_BIN, the
; deferred-exception capture (pend_capture: the $30 E1 frame and the $60 E3
; BUSY frame with WBTEMP), and FRESTORE.  Each case below drives ONE of those
; places and then:
;   1  the handler (vector 11, 55 or 48-54) logs its stack frame word and PC,
;      CORRUPTS FP0-FP7 (FMOVEM) and FPIAR before it FSAVEs -- the frame must
;      hold what the faulting instruction left, not what the registers hold
;      when it is read -- and FSAVEs to the case's first buffer (A2);
;      deferred cases FSAVE in line instead, or take the trap on an FNOP;
;   2  back in line: FPCR/FPSR cleared, FP0-FP7 and FPIAR corrupted again,
;      FRESTORE (A2), FSAVE (A3): the two frames must be identical (counted
;      at $7004, as many longwords as the frame's size byte says).
; The expect file pins every longword of every first frame and the handler
; log, as the unit produced them before the frame registers were restructured
; (AP040_XC7, findings xc7 candidate 4); the bench's clock count pins timing.
;
; diff: --cycles 200000
v_flin	equ	h_any
v_fpun	equ	h_any
v_fparith equ	h_any
v_ill	equ	unexp
v_adr	equ	unexp
v_alin	equ	unexp
v_prv	equ	unexp
v_fmt	equ	unexp
v_trp0	equ	unexp
	include	"vectors.inc"

BUF	equ	$8000			; case n: first frame at BUF+n*$100, second at +$80
LOG	equ	$7100			; handler log: frame word.w, pad.w, stacked PC.l
NCASE	equ	44

case	macro				; \1 = case number
	lea	(BUF+\1*$100).l,a2
	lea	(BUF+\1*$100+$80).l,a3
	fmovem.x (tbl).l,fp0-fp7
	fmove.l	#0,fpsr
	endm

	org	$400
start:
	clr.l	$7000			; exceptions taken
	clr.l	$7004			; round-trip mismatches
	clr.l	$7008			; cases run
	lea	(LOG).l,a4
	lea	(OUT).l,a0		; FMOVE FPn,<ea> scratch

;=================================== UNIMP ($4130) frames: vector 11
; dispatch, opclass 000: ETEMP/FPTEMP = FPm/FPn images, register tags
	case	0
	fsin.x	fp1,fp2			; both normal
	bsr	rt
	case	1
	fsin.x	fp3,fp4			; unnormal source, denormal destination (tag 4)
	bsr	rt
	case	2
	fcosh.x	fp5,fp6			; infinity, NaN
	bsr	rt
	case	3
	ftan.x	fp7,fp0			; zero source
	bsr	rt
; dispatch, FMOVECR: ETEMP 0, STAG 1, FPTEMP = FPn
	case	4
	fmovecr	#$0c,fp5
	bsr	rt
; dispatch, packed source: the split payload, STAG 7, E1
	case	5
	fsin.p	(pval).l,fp6
	bsr	rt
; F_EXEC (r_unimp): the converted memory operand, its tag
	case	6
	fsin.s	(sone).l,fp7
	bsr	rt
	case	7
	fetox.d	(dval).l,fp1
	bsr	rt
	case	8
	flogn.x	(xval).l,fp2
	bsr	rt
	case	9
	fsin.l	(lval).l,fp3
	bsr	rt
	case	10
	fsin.w	(wval).l,fp4
	bsr	rt
	case	11
	fsin.b	(bval).l,fp5
	bsr	rt
	case	12
	fsin.s	(sden).l,fp6		; single denormal: normalized, STAG 5
	bsr	rt
	case	13
	fsin.d	(dden).l,fp7		; double denormal
	bsr	rt
	case	14
	fsin.x	(xunn).l,fp0		; extended unnormal: STAG 4
	bsr	rt
	case	15
	fsin.s	(snan).l,fp1		; NaN
	bsr	rt
	case	16
	fsin.d	(dinf).l,fp2		; infinity
	bsr	rt

;=================================== BUSY ($4160) frames: vector 55
; dispatch, opclass 011: T, the register image in ETEMP and FPTEMP
	case	17
	fmove.p	fp1,(a0){#3}		; packed store: T, E1
	bsr	rt
	moveq	#-2,d0
	case	18
	fmove.p	fp2,(a0){d0}		; dynamic k
	bsr	rt
	case	19
	fmove.x	fp3,(a0)		; unnormal register: T
	bsr	rt
	case	20
	fmove.d	fp4,(a0)		; denormal register: T
	bsr	rt
; dispatch, opclass 000 hardware op, unsupported source register
	case	21
	fadd.x	fp3,fp1
	bsr	rt
	case	22
	fmul.x	fp4,fp2
	bsr	rt
; dispatch, packed memory source: split payload, STAG 7, E1
	case	23
	fadd.p	(pval).l,fp1
	bsr	rt
; F_SRC: single / double denormals (0.f under $3F80 / $3C00), extended
	case	24
	fadd.s	(sden).l,fp1
	bsr	rt
	case	25
	fmul.s	(sdenn).l,fp2		; negative
	bsr	rt
	case	26
	fmul.d	(dden).l,fp1
	bsr	rt
	case	27
	fadd.x	(xunn).l,fp1
	bsr	rt
	case	28
	fdiv.x	(xden).l,fp2
	bsr	rt
; F_EXEC: FCMP with an unsupported destination
	case	29
	fcmp.x	fp1,fp3
	bsr	rt
	case	30
	fcmp.s	(sone).l,fp4
	bsr	rt
; F_BIN: unsupported destination
	case	31
	fadd.x	fp1,fp3
	bsr	rt
	case	32
	fdiv.s	(sone).l,fp4
	bsr	rt
	case	33
	fsub.x	(snanx).l,fp3		; SNaN source, unsupported destination
	bsr	rt

;=================================== deferred exceptions (pend_capture)
; E1 arm ($4130 with E1): SNAN and a non-arith5 INEX2; DZ and OPERR trap at once
	case	34
	fmove.l	#$4000,fpcr		; SNAN
	fmove.s	(snan).l,fp2
	fsave	(a2)
	bsr	rt
	case	35
	fmove.l	#$0400,fpcr		; DZ: taken at once (vector 50), the handler FSAVEs
	fdiv.x	fp7,fp1
	bsr	rt
	case	36
	fmove.l	#$2000,fpcr		; OPERR: likewise (vector 52)
	fsqrt.x	fp2,fp6
	bsr	rt
	case	37
	fmove.l	#$0200,fpcr		; INEX2 on FSMOVE (not one of the five)
	fsmove.x fp1,fp6
	fsave	(a2)
	bsr	rt
; E3 arm ($4160, WBTEMP): INEX2 dyadic, OVFL, UNFL, monadic FSQRT
	case	38
	fmove.l	#$0200,fpcr
	fdiv.x	(three).l,fp0
	fsave	(a2)
	bsr	rt
	case	39
	fmove.l	#$1000,fpcr		; OVFL
	fmovem.x (big).l,fp6
	fmul.x	fp6,fp6
	fsave	(a2)
	bsr	rt
	case	40
	fmove.l	#$0800,fpcr		; UNFL
	fmovem.x (tiny).l,fp6
	fmul.x	fp6,fp6
	fsave	(a2)
	bsr	rt
	case	41
	fmove.l	#$0200,fpcr		; INEX2, monadic
	fsqrt.x	fp1,fp5
	fsave	(a2)
	bsr	rt
; the deferred trap DELIVERED on the next FP instruction; its handler FSAVEs
	case	42
	fmove.l	#$4000,fpcr
	fmove.s	(snan).l,fp2
	fnop
	bsr	rt
	case	43
	fmove.l	#$1000,fpcr
	fmovem.x (big).l,fp6
	fmul.x	fp6,fp6
	fnop
	bsr	rt
; a restored E1 frame with the enables ON re-arms: FSAVE right behind the
; FRESTORE extracts it again (case 34's frame, second copy at +$80)
	lea	(BUF+34*$100).l,a2
	lea	(BUF+NCASE*$100).l,a3
	fmove.l	#$4000,fpcr
	frestore (a2)
	fsave	(a3)
	fmove.l	#0,fpcr
	moveq	#12,d6
	bsr	cmpf
halt:
	bra.s	halt

;--------------------------------------------------------------- round trip
rt:	addq.l	#1,$7008
	fmove.l	#0,fpcr
	fmove.l	#0,fpsr
	fmovem.x (crp).l,fp0-fp7
	move.l	#$A5A5A5A5,d7
	fmove.l	d7,fpiar
	frestore (a2)
	fmovem.x (crp2).l,fp0-fp7
	fsave	(a3)
	moveq	#0,d6
	move.b	1(a2),d6		; frame size in bytes
	lsr.w	#2,d6			; longwords after the header = dbra count
cmpf:	move.l	(a2)+,d2
	cmp.l	(a3)+,d2
	beq.s	cmpf1
	addq.l	#1,$7004
cmpf1:	dbra	d6,cmpf
	rts

;--------------------------------------------------------------- handler
; every FP trap: log the stack frame, corrupt the registers, then FSAVE
h_any:
	addq.l	#1,$7000
	move.w	6(sp),(a4)+
	clr.w	(a4)+
	move.l	2(sp),(a4)+
	fmovem.x (crp).l,fp0-fp7
	move.l	#$5A5A5A5A,d7
	fmove.l	d7,fpiar
	fsave	(a2)
	rte

unexp:
	bra.s	unexp

;--------------------------------------------------------------- data
	cnop	0,4
tbl:	dc.l	$3FFF0000,$80000000,$00000000	; fp0  1.0
	dc.l	$40000000,$C90FDAA2,$2168C235	; fp1  pi
	dc.l	$C1230000,$F0E1D2C3,$B4A59687	; fp2  negative normal
	dc.l	$3FFF0000,$40000000,$00000001	; fp3  unnormal
	dc.l	$00000000,$00000000,$00012345	; fp4  denormal
	dc.l	$7FFF0000,$00000000,$00000000	; fp5  +infinity
	dc.l	$7FFF0000,$C0000000,$00000001	; fp6  quiet NaN
	dc.l	$80000000,$00000000,$00000000	; fp7  -0
crp:	rept	8
	dc.l	$5A5A0000,$A5A5A5A5,$5A5A5A5A	; corruption
	endr
crp2:	rept	8
	dc.l	$12340000,$87654321,$0FEDCBA9
	endr
pval:	dc.l	$00020001,$25000000,$00000000	; 1.25E+2 packed
sone:	dc.l	$3F800000			; 1.0 single
sden:	dc.l	$00400000			; single denormal 2^-127
sdenn:	dc.l	$80012345			; negative single denormal
snan:	dc.l	$7F800001			; single SNaN
lval:	dc.l	$DEADBEEF
wval:	dc.w	$8123,0
bval:	dc.b	$7F,0,0,0
dval:	dc.l	$400921FB,$54442D18		; pi double
dden:	dc.l	$80000000,$00000001		; -2^-1074
dinf:	dc.l	$FFF00000,$00000000		; -infinity
xval:	dc.l	$40010000,$B8AA3B29,$5C17F0BC	; normal extended
xunn:	dc.l	$40000000,$12345678,$9ABCDEF0	; extended unnormal
xden:	dc.l	$80000000,$00000000,$00000100	; extended denormal
snanx:	dc.l	$7FFF0000,$A0000000,$00000000	; extended SNaN
three:	dc.l	$40000000,$C0000000,$00000000	; 3.0
big:	dc.l	$7FFE0000,$80000000,$00000000
tiny:	dc.l	$00010000,$80000000,$00000000
OUT:	dcb.l	4,0

	org	BUF
	dcb.b	(NCASE+1)*$100,0
