; t_cbwaerr_pipe.s - findings/copyback/plan.md S3 (write-allocate, WALLOC):
; the allocate's line fill takes a bus error.  The store was acknowledged at
; acceptance and the core has moved on, so it cannot be restarted: the cache
; raises post_err (the late error a posted store's drain also reports) and
; the core halts; the store never reaches memory.  The 68040 reports the
; pending write in its access-error frame instead (M68040UM 8.4.6.5); here
; the core's own pending request may take the same berr, so the late-error
; halt is the honest report.
; Runs on the compat bench built with FILLCH=0 (every fill on the 16-bit
; adapter, the board's FILL_CHANNEL = 0): $F1A0 arms a one-shot bus error on
; a data read at an address, $F1A2 declares the late-error halt the expected
; end of the phase (the bench then checks post_err and that the store's
; longword in memory is unchanged).  With $F19A = 0 (no write-allocate) the
; store writes through, no read of the line happens, and the program checks
; memory and passes on its own.
; Same tables as t_cbwa_pipe.s: page 2 copyback.
; Protocol: $F100 = failing check, $F102 = $600D.
;
;   1  WALLOC: the store miss whose fill bus-errors did not halt the core
;   2  no WALLOC: the write-through store is not in memory
;   3  no WALLOC: the line's neighbour reads wrong after the store

FAILREG	equ	$F100
DONEREG	equ	$F102
PEEKA	equ	$F190
PEEKD	equ	$F192
WAMODE	equ	$F19A
DBERR	equ	$F1A0
LERREXP	equ	$F1A2
ROOT	equ	$5000
PTR	equ	$5200
PAGE	equ	$5400
CACR_ON	equ	$80008000
CM_WT	equ	$00
CM_CB	equ	$20
CM_CI	equ	$40

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

	org	0
	dc.l	$7000
	dc.l	start
	rept	254
	dc.l	unexp
	endr

	org	$400
start:	move.l	#CACR_ON,d0
	movec	d0,cacr
	cpusha	bc
	lea	ROOT,a0
	move.l	#PTR|3,(a0)
	lea	PTR,a0
	move.l	#PAGE|3,(a0)
	lea	PAGE,a0
	lea	cmtab(pc),a1
	moveq	#0,d0
	moveq	#15,d1
.pt:	move.l	d0,d2
	or.b	(a1)+,d2
	or.l	#$19,d2			; resident, U and M set
	move.l	d2,(a0)+
	add.l	#$1000,d0
	dbra	d1,.pt
	move.l	#ROOT,d0
	movec	d0,srp
	movec	d0,urp
	pflusha
	move.l	#$8000,d0
	movec	d0,tc

	move.w	#$20E0,DBERR			; a data read of $20E0 bus-errors once
	tst.w	WAMODE
	beq.s	.wt
	move.w	#$20E4,LERREXP			; the late-error halt is the expected end
	move.l	#$5555AAAA,$20E4		; store miss: the allocate's fill errs
	moveq	#-1,d0
.w:	dbra	d0,.w
	failt	1

.wt:	move.l	#$5555AAAA,$20E4		; write-through: no read of the line
	move.w	#0,DBERR
	move.w	#$20E4,d0
	bsr	peek
	cmp.l	#$5555AAAA,d1
	beq.s	.k2
	failt	2
.k2:	cmp.l	#$E0E0E0E0,$20E0
	beq.s	.k3
	failt	3
.k3:	move.w	#$600d,DONEREG
.h:	bra.s	.h

peek:	move.w	d0,PEEKA
	move.l	PEEKD,d1
	rts

fail_all:
	move.w	d7,FAILREG
	move.w	#$bad0,DONEREG
.f:	bra.s	.f

unexp:	move.w	#99,d7
	bra.s	fail_all

cmtab:	dc.b	CM_WT,CM_WT,CM_CB,CM_CB,CM_CI,CM_WT,CM_WT,CM_WT
	dc.b	CM_CB,CM_WT,CM_CB,CM_WT,CM_WT,CM_WT,CM_WT,CM_CI

	org	$20E0
	dc.l	$E0E0E0E0,$E4E4E4E4
