* Minimal OS Rom - mostly done to ensure that simulation works

* Set up SSP - SegaProgrammingFAQ.txt line 329
*	dc.l	0xffffff00
*	dc.l	0x00000000
	dc.l	0x1337b00b
* Entry point
	dc.l	Init
Init:

* Disable interrupts (?)	
	move.w  #0x2700, %sr

	move.l	#0x00000000, %d0
	move.l	%d0, %d1
	move.l	%d0, %d2
	move.l	%d0, %d3
	move.l	%d0, %d4
	move.l	%d0, %d5
	move.l	%d0, %d6
	move.l	%d0, %d7
	movea.l	%d0, %a0
	movea.l	%d0, %a1
	movea.l	%d0, %a2
	movea.l	%d0, %a3
	movea.l	%d0, %a4
	movea.l	%d0, %a5
	movea.l	%d0, %a6
* A7 = SSP, already set
* Set USP to 0x000000
*	move.l	#0xffffff00, %a6
	move.l	%a6, %usp
	
* Copy the Boot sequence into RAM
	lea 	Boot, %a0
	lea 	0xffffff00, %a1
	move.l  #BootEnd, %d0
	move.l  #Boot, %d1
	sub.l   %d1, %d0
Copy:
	move.w  (%a0)+,(%a1)+
	subq.w  #2, %d0
	bne     Copy
* Jump to Boot sequence
	lea 	0xffffff00, %a0
	jmp		(%a0)
	
Boot:
* Disable OS ROM - SegaProgrammingFAQ.txt line 321
	lea		0x00a14100, %a0
	move.w	#0x0001, (%a0)

* Set version register according to cartridge information
* gp-062203-src\io.c line 96
	lea		0x000001f0, %a0
	move.b	(%a0), %d0
	cmp.b 	#'J', %d0
	beq		Ver00
	cmp.b	#'E', %d0
	beq		VerC0
	cmp.b	#'A', %d0
	beq		VerC0
	cmp.b	#'B', %d0
	beq		VerC0
Ver80:
	move.b	#0xa0, %d0
	bra		SetVer
Ver00:
	move.b	#0x20, %d0
	bra		SetVer
VerC0:
	move.b	#0xe0, %d0
SetVer:
	lea		0x00a10001, %a0
	move.b	%d0, (%a0)
* Set up SSP according to cartridge's header
	lea		0x00000000, %a0
	movea.l	(%a0), %a7
* Jump to cartridge's entry point
	lea		0x00000004, %a0
	movea.l	(%a0), %a0
	jmp		(%a0)
BootEnd:
