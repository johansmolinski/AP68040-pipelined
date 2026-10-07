; t_cb_pipe.s - findings/copyback/plan.md S0: the copyback data cache.
; Translation on, 4K identity pages; the work pages are copyback (CM=01).
; Memory behind the caches is read through the bench's peek registers
; ($F190 address, $F192 the longword), on a cache-inhibited page.  With
; COPYBACK ($F196 = 1) a store hit stays in the cache until it is pushed;
; without it every store writes through, and the program checks that too:
; only case 1's first peek depends on the mode.  Runs under
; tb_ap040_pipe_compat.v.  Protocol: $F100 = failing check, $F102 = $600D.
;
;  pages: 0-1 code (CM=00), 2-3 work (copyback), 4 = physical page 2,
;  cache-inhibited (the alias), 5 tables (CM=00), 6-7 stack (CM=00),
;  8 code written through copyback, 9 write-through work, F registers (CI)
;
;   1  a store hit: memory unchanged until CPUSHA (copyback), then new
;   2  eviction: four more lines in the set push the dirty one
;   3  CINVA over a dirty line writes it back (plan revision 3)
;   4  a misaligned store over a dirty line: memory and cache agree
;   5  a cache-inhibited read (the alias) of a dirty line sees the store
;   6  MOVE16 from a dirty line
;   7  a DMA write to another line of a dirty line's set keeps the line
;   8  code stored through copyback runs after CPUSHA
;   9  a write-through page writes through
;  10  a store miss writes through (no write-allocate); with WALLOC ($F19A
;      = 1) it allocates instead: memory unchanged until CPUSHA, then new
;      (t_cbwa_pipe tests the allocate itself)
;  11  all four ways of one set dirty ($2080/$2480/$2880/$2C80), CPUSHA
;      writes every one back
;  12  three ways of one set dirty ($2090/$2490/$2890), then a
;      cache-inhibited read in that set: every dirty way written back first
;  13  MOVE16 of a whole line, four different longwords: from a dirty
;      copyback line ($20A0 -> $30A0) and from a write-through line
;      ($90A0 -> $90C0), every longword in its place

FAILREG	equ	$F100
DONEREG	equ	$F102
PEEKA	equ	$F190
PEEKD	equ	$F192
CBMODE	equ	$F196
WAMODE	equ	$F19A
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
	; tables: 16 identity pages with their cache modes
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
	or.l	#$19,d2			; resident, U and M set: the walker never
					; writes a descriptor (its snoop would clear
					; set 0, where $2000 lives)
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

; 1: a store hit stays in the cache until it is pushed
	move.l	$2000,d0			; the line becomes resident
	cmp.l	#$A0A0A0A0,d0
	beq.s	.c1a
	failt	1
.c1a:	move.l	#$11111111,$2000
	cmp.l	#$11111111,$2000		; the cache has it
	beq.s	.c1b
	failt	11
.c1b:	tst.w	CBMODE
	beq.s	.c1wt
	chkpk	$2000,$A0A0A0A0,21		; copyback: memory still old
	bra.s	.c1p
.c1wt:	chkpk	$2000,$11111111,31		; write-through: memory new
.c1p:	cpusha	dc
	chkpk	$2000,$11111111,41

; 2: eviction pushes the dirty line (set $01: $2010, $2410, $2810, $2C10, $3010)
	move.l	$2010,d0
	move.l	#$22222222,$2010
	move.l	$2410,d0
	move.l	$2810,d0
	move.l	$2C10,d0
	move.l	$3010,d0
	chkpk	$2010,$22222222,2
	cmp.l	#$22222222,$2010
	beq.s	.c2
	failt	12

; 3: CINVA over a dirty line writes it back first
.c2:	move.l	$2020,d0
	move.l	#$33333333,$2020
	cinva	dc
	chkpk	$2020,$33333333,3
	cmp.l	#$33333333,$2020
	beq.s	.c3
	failt	13

; 4: a misaligned store over a dirty line
.c3:	move.l	$2030,d0
	move.l	#$44444444,$2030		; dirty
	move.l	#$55667788,$2031		; misaligned: cannot merge
	cmp.l	#$44556677,$2030
	beq.s	.c4a
	failt	4
.c4a:	cmp.l	#$88A4A4A4,$2034
	beq.s	.c4b
	failt	14
.c4b:	cpusha	dc
	chkpk	$2030,$44556677,24
	chkpk	$2034,$88A4A4A4,34

; 5: a cache-inhibited read of a dirty line sees the store
	move.l	$2040,d0
	move.l	#$66666666,$2040
	cmp.l	#$66666666,$4040		; the inhibited alias of $2040
	beq.s	.c5
	failt	5

; 6: MOVE16 from a dirty line
.c5:	move.l	$2050,d0
	move.l	#$77777777,$2050
	lea	$2050,a0
	lea	$3050,a1
	move16	(a0)+,(a1)+
	cmp.l	#$77777777,$3050
	beq.s	.c6a
	failt	6
.c6a:	cpusha	dc
	chkpk	$3050,$77777777,16

; 7: a DMA write to another line of the set leaves the dirty line alone
	move.l	$2460,d0			; set $06, way 0 (the victim a
						; row clear resets the pointer to)
	move.l	$2060,d0
	move.l	#$88888888,$2060		; dirty, set $06, way 1
	move.w	#$1C60,$F1E4			; the DMA agent: $1C60 (set $06)
	move.w	#$5A5A,$F1E6
	move.w	#0,$F1EA			; with its snoop
	move.w	#4,$F1E8
	moveq	#40,d0
.w7:	dbra	d0,.w7
	cmp.l	#$88888888,$2060
	beq.s	.c7a
	failt	7
.c7a:	cpusha	dc
	chkpk	$2060,$88888888,17
	chkpk	$1C60,$5A5AC6C6,27

; 8: code stored through copyback runs after CPUSHA
	move.l	#$702A4E75,$8000		; moveq #42,d0 ; rts
	cpusha	bc
	moveq	#0,d0
	jsr	$8000
	cmp.l	#42,d0
	beq.s	.c8
	failt	8

; 9: a write-through page writes through
.c8:	move.l	$9000,d0
	move.l	#$99999999,$9000
	chkpk	$9000,$99999999,9

; 10: a store miss writes through -- or, with write-allocate, stays in the
; cache (dirty) until it is pushed
	move.l	#$ABABABAB,$2070		; never read: not resident
	tst.w	WAMODE
	beq.s	.c10wt
	chkpk	$2070,$A8A8A8A8,110		; allocated: memory still old
	cpusha	dc
.c10wt:	chkpk	$2070,$ABABABAB,10

; 11: four dirty ways in one set
	move.l	$2080,d0
	move.l	$2480,d0
	move.l	$2880,d0
	move.l	$2C80,d0
	move.l	#$B0B0B001,$2080
	move.l	#$B0B0B002,$2480
	move.l	#$B0B0B003,$2880
	move.l	#$B0B0B004,$2C80
	cpusha	dc
	chkpk	$2080,$B0B0B001,111
	chkpk	$2480,$B0B0B002,112
	chkpk	$2880,$B0B0B003,113
	chkpk	$2C80,$B0B0B004,114
	cmp.l	#$B0B0B003,$2880
	beq.s	.c11
	failt	115
.c11:
; 12: three dirty ways, then an inhibited read in the set
	move.l	$2090,d0
	move.l	$2490,d0
	move.l	$2890,d0
	move.l	#$C0C0C001,$2090
	move.l	#$C0C0C002,$2490
	move.l	#$C0C0C003,$2890
	cmp.l	#$C0C0C001,$4090		; the inhibited alias of $2090
	beq.s	.c12a
	failt	12
.c12a:	chkpk	$2490,$C0C0C002,122
	chkpk	$2890,$C0C0C003,123
	cmp.l	#$C0C0C002,$2490
	beq.s	.c12b
	failt	124
.c12b:
; 13: MOVE16, four different longwords
	lea	$20A0,a0
	move.l	(a0),d0				; resident
	move.l	#$D0D0D000,(a0)+		; dirty
	move.l	#$D0D0D001,(a0)+
	move.l	#$D0D0D002,(a0)+
	move.l	#$D0D0D003,(a0)+
	lea	$20A0,a0
	lea	$30A0,a1
	move16	(a0)+,(a1)+
	lea	$30A0,a1
	move.l	#$D0D0D000,d1
	moveq	#3,d3
.m13:	cmp.l	(a1)+,d1
	beq.s	.n13
	failt	13
.n13:	addq.l	#1,d1
	dbra	d3,.m13
	lea	$90A0,a0
	move.l	#$E0E0E000,(a0)+		; write-through page
	move.l	#$E0E0E001,(a0)+
	move.l	#$E0E0E002,(a0)+
	move.l	#$E0E0E003,(a0)+
	lea	$90A0,a0
	lea	$90C0,a1
	move16	(a0)+,(a1)+
	lea	$90C0,a1
	move.l	#$E0E0E000,d1
	moveq	#3,d3
.w13:	cmp.l	(a1)+,d1
	beq.s	.v13
	failt	133
.v13:	addq.l	#1,d1
	dbra	d3,.w13
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
	dc.b	CM_CB,CM_WT,CM_WT,CM_WT,CM_WT,CM_WT,CM_WT,CM_CI

; the work lines' initial memory
	org	$1C60
	dc.l	$C6C6C6C6
	org	$2000
	dc.l	$A0A0A0A0
	org	$2010
	dc.l	$A1A1A1A1
	org	$2020
	dc.l	$A2A2A2A2
	org	$2030
	dc.l	$A3A3A3A3,$A4A4A4A4
	org	$2040
	dc.l	$A5A5A5A5
	org	$2050
	dc.l	$A6A6A6A6,0,0,0
	org	$2060
	dc.l	$A7A7A7A7
	org	$2070
	dc.l	$A8A8A8A8
	org	$9000
	dc.l	$A9A9A9A9
