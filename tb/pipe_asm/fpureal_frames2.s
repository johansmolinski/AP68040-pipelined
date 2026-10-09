; fpureal_frames2.s - two more frame sources fpureal_frames.s leaves unseen
; (found by the AP040_XC7 frame mutants, findings xc7 candidate 4):
;   0  a deferred E3 (INEX2 on FSADD) whose DESTINATION is a zero: the
;      BUSY frame's DTAG is the shadow's (sh_dtag = 1), not 0 -- every
;      dyadic E3 case there had a normal destination;
;   1  a deferred exception raised by a BUSY command RESUMED from the frame
;      (FRESTORE with CU_SAVEPC = $FE, opclass 000 FADD, INEX2 enabled):
;      the new frame's CMDREG1B is the restored command word (opclass 000),
;      not the unit's own {010, fmt, FPn, op} -- the two agree everywhere
;      else.
; Each frame is FSAVEd in line, FRESTOREd with the enables off, FSAVEd again
; and compared (mismatches at $7004); the expect file pins the first frames
; as the unit produced them at 9d21f44.
;
; diff: --cycles 60000
v_flin	equ	unexp
v_fpun	equ	unexp
v_fparith equ	h_arith
v_ill	equ	unexp
v_adr	equ	unexp
v_alin	equ	unexp
v_prv	equ	unexp
v_fmt	equ	unexp
v_trp0	equ	unexp
	include	"vectors.inc"

BUF	equ	$8000

	org	$400
start:
	clr.l	$7000			; arithmetic traps taken
	clr.l	$7004			; round-trip mismatches
;------------------------------------- 0: E3, dyadic, zero destination
	lea	(BUF).l,a2
	lea	(BUF+$80).l,a3
	fmovem.x (pi).l,fp1
	fmove.l	#0,fp7			; +0
	fmove.l	#$0200,fpcr		; INEX2
	fsadd.x	fp1,fp7			; pi rounded to single: inexact
	fsave	(a2)
	bsr	rt
;------------------------------------- 1: E3 from a resumed BUSY command
	lea	(BUF+$100).l,a2
	lea	(BUF+$180).l,a3
	lea	(busy).l,a4
	fmove.l	#$0200,fpcr
	frestore (a4)			; FADD ETEMP to FPTEMP -> FP2: 1 + 2^-70, inexact
	fsave	(a2)
	bsr	rt
halt:
	bra.s	halt

rt:	fmove.l	#0,fpcr
	fmove.l	#0,fpsr
	frestore (a2)
	fsave	(a3)
	moveq	#0,d6
	move.b	1(a2),d6
	lsr.w	#2,d6
cmpf:	move.l	(a2)+,d2
	cmp.l	(a3)+,d2
	beq.s	cmpf1
	addq.l	#1,$7004
cmpf1:	dbra	d6,cmpf
	rts

h_arith:
	addq.l	#1,$7000
	rte

unexp:
	bra.s	unexp

	cnop	0,4
pi:	dc.l	$40000000,$C90FDAA2,$2168C235
busy:	dc.l	$41600000,$00000000,$FE000000	; CU_SAVEPC $FE
	dcb.l	13,0
	dc.l	$01220000		; 16 CMDREG1B: opclass 000, FP2, FADD
	dc.l	0,0
	dc.l	$3FFF0000,$80000000,$00000000	; 19-21 FPTEMP 1.0
	dc.l	$3FB90000,$80000000,$00000000	; 22-24 ETEMP 2^-70

	org	BUF
	dcb.b	$200,0
