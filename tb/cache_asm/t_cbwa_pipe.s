; t_cbwa_pipe.s - findings/copyback/plan.md S3: write-allocate on a copyback
; store miss (M68040UM 4.4.2: a write miss to a copyback page reads the line
; into the cache, merges the data and leaves the line dirty).
; Same memory map and tables as t_cb_pipe.s: pages 2-3 and 8 copyback
; (CM=01), page 4 = physical page 2 cache-inhibited (the alias), page 9
; write-through, page A copyback (the walker case), page F the registers.
; $F19A = 1 when the core allocates on a copyback store miss (WALLOC):
; only the peeks that look at memory BEFORE a push depend on it; every
; other check holds for a write-through store miss too.
; Protocol: $F100 = failing check, $F102 = $600D.
;
;   1  a longword store miss: memory unchanged until CPUSHA (WALLOC), the
;      store and its line's neighbour readable from the cache, both in
;      memory after the push
;   2  a byte and a word store miss merge into the fetched line, byte-exact
;   3  a store miss whose victim is dirty: the victim is pushed first
;   5  a store miss to a write-through page still writes through; the
;      second one (15) posted, the first having proved the page writable
;   6  a misaligned store miss to a copyback page still writes through
;   7  a store miss followed at once by a load of its line's neighbour
;   8  four store misses into one line (the stack-push pattern)
;   9  the set's other ways survive the allocate: a dirty neighbour keeps
;      its tag while the next request names another set
;  10  the table walker waits for the allocate (the next access misses
;      the ATC while the line is being fetched)
;  13  CPUSHA right after a store miss writes the allocated line back
;  14  a cache-inhibited read of the alias right after a store miss
;  15  a read miss in the allocated line's set right after the store miss
;  16  an instruction-cache miss right before the store miss
; 100+N / 200+N / 300+N / 400+N  a chipset snoop of the set N clocks after
;      the store is armed (N = 0..59): the store is seen by the next read, is
;      in memory after CPUSHA, the set's dirty neighbour survives, the
;      other line of the set is untouched, and so is $0040 (500+N: tag 0 +
;      set $04, where a dirty way whose row CPUSHA zeroed would be pushed)

FAILREG	equ	$F100
DONEREG	equ	$F102
PEEKA	equ	$F190
PEEKD	equ	$F192
CBMODE	equ	$F196
WAMODE	equ	$F19A
DMAA	equ	$F1E4
DMAD	equ	$F1E6
DMAN	equ	$F1E8
DMANS	equ	$F1EA
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

; peek \1 (a 16-bit address) must hold \2, else fail \3
chkpk	macro
	move.w	#\1,d0
	bsr	peek
	cmp.l	#\2,d1
	beq.s	.k\@
	failt	\3
.k\@:
	endm

; the cache (a read) must give \2 at \1, else fail \3
chkrd	macro
	cmp.l	#\2,\1
	beq.s	.r\@
	failt	\3
.r\@:
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
	move.l	#$2000|CM_CI|$19,PAGE+4*4	; page 4 = physical page 2, inhibited
	move.l	#ROOT,d0
	movec	d0,srp
	movec	d0,urp
	pflusha
	move.l	#$8000,d0
	movec	d0,tc

; 1: a longword store miss allocates
	move.l	#$11111111,$2000		; never read: a miss
	tst.w	WAMODE
	beq.s	.c1wt
	chkpk	$2000,$A0A0A0A0,1		; allocated: memory still old
	bra.s	.c1
.c1wt:	chkpk	$2000,$11111111,51		; write-through: memory new
.c1:	chkrd	$2000,$11111111,11
	chkrd	$2004,$A1A1A1A1,21		; the line's neighbour came with the fill
	cpusha	dc
	chkpk	$2000,$11111111,31
	chkpk	$2004,$A1A1A1A1,41

; 2: byte and word store misses, byte-exact
	move.b	#$5A,$2011
	move.w	#$1234,$2022
	chkrd	$2010,$B05AB0B0,2
	chkrd	$2020,$C0C01234,12
	cpusha	dc
	chkpk	$2010,$B05AB0B0,22
	chkpk	$2020,$C0C01234,32

; 3: a store miss whose victim is dirty (set $03: four dirty ways, then $3030)
	move.l	$2030,d0
	move.l	#$33333301,$2030
	move.l	$2430,d0
	move.l	#$33333302,$2430
	move.l	$2830,d0
	move.l	#$33333303,$2830
	move.l	$2C30,d0
	move.l	#$33333304,$2C30
	move.l	#$33333305,$3030		; the fifth line: victim way 0 ($2030)
	tst.w	WAMODE
	beq.s	.c3wt
	chkpk	$2030,$33333301,3		; the victim was pushed
	chkpk	$3030,$A5A5A5A5,13		; the new line is dirty, not in memory
	bra.s	.c3
.c3wt:	chkpk	$3030,$33333305,53
.c3:	chkrd	$3030,$33333305,23
	cpusha	dc
	chkpk	$3030,$33333305,33
	chkpk	$2430,$33333302,43

; 5: a store miss to a write-through page writes through.  The first store
;    to page 9 may be synchronous (SB_MMU posts only into a page a store has
;    proved writable); the second is posted like a copyback store and still
;    writes through (15).  (A posted write-through store goes out on the
;    master port in its first C_PASS clock, so even a core that allocated
;    it as well would show the right memory here: the white-box checker's
;    "store already issued" rule is what catches that.)
	move.l	#$99999999,$9010
	chkpk	$9010,$99999999,5
	move.l	#$98989898,$9020
	chkpk	$9020,$98989898,15
	cmp.l	#$98989898,$9020
	beq.s	.c5
	failt	25
.c5:

; 6: a misaligned store miss to a copyback page writes through (two pieces)
	move.l	#$56789ABC,$2052
	chkpk	$2050,$E0E05678,6
	chkpk	$2054,$9ABCE1E1,16
	chkrd	$2050,$E0E05678,26

; 7: a store miss, then at once a load of the line's neighbour
	move.l	#$77777777,$2060
	move.l	$2064,d0
	cmp.l	#$F1F1F1F1,d0
	beq.s	.c7
	failt	7
.c7:	chkrd	$2060,$77777777,17

; 8: four store misses into one line
	lea	$2070,a0
	move.l	#1,(a0)+
	move.l	#2,(a0)+
	move.l	#3,(a0)+
	move.l	#4,(a0)+
	tst.w	WAMODE
	beq.s	.c8wt
	chkpk	$2070,$70707070,8
	bra.s	.c8
.c8wt:	chkpk	$2070,1,58
.c8:	cpusha	dc
	chkpk	$2070,1,18
	chkpk	$2074,2,28
	chkpk	$2078,3,38
	chkpk	$207C,4,48

; 9: the set's other ways survive: set $08, $2480 dirty (way 0), $2880
;    clean (way 1), the store miss $2080 (way 2) while the next request
;    names set $0F
	move.l	$2480,d0
	move.l	#$84848401,$2480
	move.l	$2880,d0
	move.l	#$99999999,$2080
	move.l	$20F0,d0			; another set, held during the allocate
	cmp.l	#$0F0F0F0F,d0
	beq.s	.c9a
	failt	49
.c9a:	chkrd	$2480,$84848401,9
	chkrd	$2880,$88888888,19
	cpusha	dc
	chkpk	$2480,$84848401,29
	chkpk	$2080,$99999999,39
	chkpk	$2880,$88888888,59

; 10: the table walker waits for the allocate
	pflusha
	move.l	#$AAAA0101,$2090		; page 2 walked, then the store misses
	move.l	$A000,d0			; page A: an ATC miss while the line is fetched
	cmp.l	#$AAAA0001,d0
	beq.s	.c10
	failt	60
.c10:	chkrd	$2090,$AAAA0101,61
	cpusha	dc
	chkpk	$2090,$AAAA0101,62

; 13: CPUSHA right after the store miss
	move.l	#$CCCCCCCC,$20A0
	cpusha	dc
	chkpk	$20A0,$CCCCCCCC,63

; 14: a cache-inhibited read of the alias right after the store miss
	move.l	#$BBBBBBBB,$20B0
	move.l	$40B0,d0
	cmp.l	#$BBBBBBBB,d0
	beq.s	.c14
	failt	64

; 15: a read miss in the same set right after the store miss
.c14:	move.l	#$ABABABAB,$20C0
	move.l	$24C0,d0
	cmp.l	#$C4C4C4C4,d0
	beq.s	.c15
	failt	65
.c15:	chkrd	$20C0,$ABABABAB,66

; 16: an instruction-cache miss right before the store miss (the fill
;     before the allocate wrote the INSTRUCTION bank: the allocate's beats
;     must go to the data bank)
.c16:	cinva	ic
	move.l	#$16161616,$20D0
	chkrd	$20D4,$D4D4D4D4,67
	chkrd	$20D0,$16161616,68
	cpusha	dc
	chkpk	$20D0,$16161616,69
	chkpk	$20D4,$D4D4D4D4,70

; 100+N: a snoop of the set N clocks after the store is armed.  The line
; under test alternates between $2040 and $3040 (set $04, two tags): the
; victim way's stale tag is then the OTHER line's, so a dirty bit set on a
; line that was never validated would push this round's data over the other
; line's memory (400+N).
	moveq	#0,d5				; N
	move.l	#$10000000,d6			; the store's value, changes per round
loop:	cpusha	dc				; clean slate
	move.l	$2440,d0
	move.l	d6,$2440			; a dirty neighbour in set $04 (way 0)
	lea	$2040,a2			; this round's line and the other one
	lea	$3040,a3
	btst	#0,d5
	beq.s	.pick
	exg	a2,a3
.pick:	move.w	#$1C40,DMAA			; the DMA agent: $1C40 (set $04)
	move.w	#$5A5A,DMAD
	move.w	#0,DMANS			; with its snoop
	move.w	d5,DMAN				; N clocks after this write
	move.l	d6,(a2)				; the store miss under test
	cmp.l	(a2),d6
	beq.s	.s1
	move.w	d5,d7
	add.w	#100,d7
	bra	fail_all
.s1:	cmp.l	$2440,d6
	beq.s	.s2
	move.w	d5,d7
	add.w	#300,d7
	bra	fail_all
.s2:	moveq	#40,d0
.w:	dbra	d0,.w				; let the agent fire if it has not yet
	cpusha	dc
	move.w	a2,d0
	bsr	peek
	cmp.l	d1,d6
	beq.s	.s3
	move.w	d5,d7
	add.w	#200,d7
	bra	fail_all
.s3:	move.w	a3,d0				; the other line: last round's value
	bsr	peek
	move.l	d6,d2
	sub.l	#$01010101,d2
	cmp.l	d1,d2
	beq.s	.s4
	move.w	d5,d7
	add.w	#400,d7
	bra	fail_all
.s4:	move.w	#$0040,d0			; tag 0, set $04: where a never-validated
	bsr	peek				; way (its row zeroed by CPUSHA) would be
	cmp.l	#unexp,d1			; pushed to -- vectors 16-19 must hold
	beq.s	.s5
	move.w	d5,d7
	add.w	#500,d7
	bra	fail_all
.s5:	add.l	#$01010101,d6
	addq.w	#1,d5
	cmp.w	#60,d5
	bne	loop

	move.w	#$600d,DONEREG
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

; the work lines' initial memory
	org	$1C40
	dc.l	$C6C6C6C6
	org	$2000
	dc.l	$A0A0A0A0,$A1A1A1A1
	org	$2010
	dc.l	$B0B0B0B0
	org	$2020
	dc.l	$C0C0C0C0
	org	$2030
	dc.l	$A3A3A3A3
	org	$2040
	dc.l	$D0D0D0D0
	org	$2050
	dc.l	$E0E0E0E0,$E1E1E1E1
	org	$2060
	dc.l	$F0F0F0F0,$F1F1F1F1
	org	$2070
	dc.l	$70707070,$71717171,$72727272,$73737373
	org	$2080
	dc.l	$80808080
	org	$2090
	dc.l	$90909090
	org	$20A0
	dc.l	$A0A0A0A1
	org	$20B0
	dc.l	$B0B0B0B1
	org	$20C0
	dc.l	$C0C0C0C1
	org	$20D0
	dc.l	$D0D0D0D1,$D4D4D4D4
	org	$20F0
	dc.l	$0F0F0F0F
	org	$2430
	dc.l	$A4A4A4A4
	org	$2440
	dc.l	$D1D1D1D1
	org	$3040
	dc.l	$0EFEFEFF			; = round 0's value - $01010101
	org	$2480
	dc.l	$84848484
	org	$24C0
	dc.l	$C4C4C4C4
	org	$2830
	dc.l	$A6A6A6A6
	org	$2880
	dc.l	$88888888
	org	$2C30
	dc.l	$A7A7A7A7
	org	$3030
	dc.l	$A5A5A5A5
	org	$9010
	dc.l	$91919191
	org	$9020
	dc.l	$92929292
	org	$A000
	dc.l	$AAAA0001
