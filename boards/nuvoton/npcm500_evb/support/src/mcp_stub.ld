/*
 * mcp_stub runs from the top 4KB of MCU SRAM (0x3000_0000, 640KB), clear of the MCU
 * runtime that ROM loads at 0x3000_0000, so no stale I-cache lines for code we load
 * without fence.i (progbufsize=0). The image buffer is 0x3000_0000 .. STUB_ADDR.
 */
ENTRY(_start)

MEMORY
{
	STUB (rwx) : ORIGIN = 0x3009F000, LENGTH = 0x1000
}

SECTIONS
{
	.text : {
		KEEP(*(.text.entry))
		*(.text .text.*)
		*(.rodata .rodata.*)
		*(.srodata .srodata.*)
	} > STUB

	/DISCARD/ : { *(.data .data.* .sdata .sdata.* .bss .bss.* .sbss .sbss.* .comment) }

	__stack_top = ORIGIN(STUB) + LENGTH(STUB);
}
