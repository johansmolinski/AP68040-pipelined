; Pipelined AP68040: a released FP operation and an interrupt's entry, and the
; mask an interrupt handler runs with.  Runs under tb_ap040_pipe_compat.v.
; Protocol: $F100 word = failing check, $F102 = $600D / $BAD0.
;
;   1-2  a level-2 interrupt swept across a released FDIV (1000/3, the full
;        divide) and 0..NFILL integer instructions behind it, cache-hot: the
;        unit's one-clock `done` can land in the EX-redirect clock of the
;        interrupt's entry.  Lost there, fp_bg stays set and the next FP
;        instruction (the FMOVE.L FP0,D0 reading the quotient) waits for
;        ever: the run ends at the bench's timeout.  1 = quotient,
;        2 = interrupt count.
;   3-4  the same with INEX2 enabled: every FDIV ends in its deferred
;        exception (vector 49) instead, in the same clock.  4 = exactly one
;        exception per divide (3 is not used).
;   5    a level-1 request withdrawn after a swept width ($F14C): when the
;        interrupt is taken, its handler runs with mask 1.  The mask came
;        from the live level, so a request withdrawn in the clock the vector
;        was taken left mask 0 (a core-alone bench hit it; in this bench's
;        timing the widths do not reach that clock, 6 does).
;   6    level 2 falling back to level 1 after a swept width ($F150): each
;        handler runs with its own level's mask (the level-2 handler ran
;        with mask 1 when the fallback met the take clock).
;
; Before the fix of 1-4 the run wedges at the first lost `done` and ends at
; the bench's timeout (run_pipe_tests.sh gives it 1M clocks; it passes in
; about 120k).

FAILREG		equ	$F100
DONEREG		equ	$F102
IPLREG		equ	$F110
IPLDLY		equ	$F148
IPLPULSE	equ	$F14C
IPLSTEP		equ	$F150
cnt_int1	equ	$3600
cnt_int2	equ	$3602
cnt_inex	equ	$3604
badmask		equ	$3606	; first wrong mask a handler saw: $8000 | level << 4 | mask
cnt_exp		equ	$3608	; interrupts expected so far
NDLY		equ	48	; delays 1..NDLY clocks
NFILL		equ	3	; 0..NFILL integer instructions behind the FDIV
NWID		equ	48	; widths 1..NWID clocks

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm
chkl	macro
	cmp.l	#\2,\1
	beq.s	ok\@
	failt	\3
ok\@:
	endm

	org	0
	dc.l	$3400
	dc.l	start
	rept	22
	dc.l	unexp		; 2-23
	endr
	dc.l	unexp		; 24 spurious (never: autovectors)
	dc.l	h_l1		; 25
	dc.l	h_l2		; 26
	rept	22
	dc.l	unexp		; 27-48
	endr
	dc.l	h_inex		; 49 FP inexact
	rept	206
	dc.l	unexp		; 50-255
	endr

	org	$400
start:
	move.w	#$2700,sr
	move.l	#$80008000,d0
	movec	d0,cacr		; cache-hot: instruction boundaries every clock or two
	clr.l	(cnt_int1).l	; cnt_int1, cnt_int2
	clr.l	(cnt_inex).l	; cnt_inex, badmask
	move.l	#1000,d0
	fmove.l	d0,fp2
	moveq	#3,d0
	fmove.l	d0,fp1

;------------------------------------------------------ 1-4 FDIV under interrupts
	moveq	#0,d5			; 0: done, 1: INEX2 enabled
pass_loop:
	tst.w	d5
	beq.s	pass_go
	fmove.l	#$0200,fpcr		; INEX2: 1000/3 is inexact
pass_go:
	move.w	#1,(cnt_exp).l
	clr.w	(cnt_int2).l
	clr.w	(cnt_inex).l
	move.w	#$2000,sr
	moveq	#0,d2			; integer instructions behind the FDIV
fill_loop:
	lea	fill_end(pc),a1
	sub.w	d2,a1
	sub.w	d2,a1			; ADDQ.L is one word
	moveq	#1,d4
dly_loop:
	move.w	d4,(IPLDLY).l		; level 2 after d4 clocks
	fmove.x	fp2,fp0
	fdiv.x	fp1,fp0			; released: runs on in the background
	jmp	(a1)
	rept	NFILL
	addq.l	#1,d3
	endr
fill_end:
	fmove.l	fp0,d0			; waits for the divide (for ever, if its end was lost)
	tst.w	d5
	bne.s	dly_wait
	chkl	d0,333,1
dly_wait:
	move.w	(cnt_int2).l,d0
	cmp.w	(cnt_exp).l,d0		; exactly one interrupt per armed delay
	bne.s	dly_wait
	addq.w	#1,(cnt_exp).l
	addq.w	#1,d4
	cmp.w	#NDLY,d4
	bls.s	dly_loop
	addq.w	#1,d2
	cmp.w	#NFILL,d2
	bls.s	fill_loop
	move.w	#$2700,sr
	moveq	#0,d0
	move.w	(cnt_int2).l,d0
	chkl	d0,NDLY*(NFILL+1),2
	tst.w	d5
	beq.s	pass_next
	moveq	#0,d0
	move.w	(cnt_inex).l,d0
	chkl	d0,NDLY*(NFILL+1),4
pass_next:
	fmove.l	#0,fpcr
	addq.w	#1,d5
	cmp.w	#1,d5
	bls	pass_loop

;------------------------------------------------------ 5 a withdrawn request
	move.w	#$2000,sr
	moveq	#1,d5
pls_loop:
	move.w	d5,d6
	lsl.w	#8,d6
	ori.w	#1,d6			; width:level 1
	move.w	d6,(IPLPULSE).l
	moveq	#40,d1
pls_spin:
	dbra	d1,pls_spin		; taken or withdrawn by now
	addq.w	#1,d5
	cmp.w	#NWID,d5
	bls.s	pls_loop
	move.w	#$2700,sr
	move.w	#0,(IPLREG).l
	tst.w	(badmask).l
	beq.s	pls_ok
	failt	5
pls_ok:

;------------------------------------------------------ 6 a downgraded request
	move.w	#$2000,sr
	moveq	#1,d5
stp_loop:
	move.w	d5,d6
	lsl.w	#8,d6
	ori.w	#$12,d6			; width:levels -- 2 now, 1 after width
	move.w	d6,(IPLSTEP).l
	moveq	#40,d1
stp_spin:
	dbra	d1,stp_spin		; level 1 is released by its handler
	addq.w	#1,d5
	cmp.w	#NWID,d5
	bls.s	stp_loop
	move.w	#$2700,sr
	move.w	#0,(IPLREG).l
	tst.w	(badmask).l
	beq.s	stp_ok
	failt	6
stp_ok:

	move.w	#$600D,(DONEREG).l
done:	bra.s	done

;------------------------------------------------------ handlers
; Each checks its own mask first (MOVE from SR: the mask the entry set),
; then releases the lines.  None touches the FPU.
h_l1:
	move.w	d0,-(sp)
	move.w	sr,d0
	and.w	#$0700,d0
	cmp.w	#$0100,d0
	beq.s	h_l1_ok
	lsr.w	#8,d0
	or.w	#$8010,d0
	tst.w	(badmask).l
	bne.s	h_l1_ok
	move.w	d0,(badmask).l
h_l1_ok:
	move.w	#0,(IPLREG).l
	addq.w	#1,(cnt_int1).l
	move.w	(sp)+,d0
	rte

h_l2:
	move.w	d0,-(sp)
	move.w	sr,d0
	and.w	#$0700,d0
	cmp.w	#$0200,d0
	beq.s	h_l2_ok
	lsr.w	#8,d0
	or.w	#$8020,d0
	tst.w	(badmask).l
	bne.s	h_l2_ok
	move.w	d0,(badmask).l
h_l2_ok:
	move.w	#0,(IPLREG).l
	addq.w	#1,(cnt_int2).l
	move.w	(sp)+,d0
	rte

; the deferred INEX2 of a released FDIV: pre-instruction, format $0
h_inex:
	move.w	d6,-(sp)
	move.w	8(sp),d6
	cmp.w	#$00C4,d6		; format $0, vector 49 offset
	bne.s	unexp
	addq.w	#1,(cnt_inex).l
	move.w	(sp)+,d6
	rte

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
h1:	bra.s	h1
unexp:
	move.w	#$0099,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
h2:	bra.s	h2
