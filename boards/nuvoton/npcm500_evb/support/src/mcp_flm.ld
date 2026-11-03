/*
 * SEGGER Flash Loader / CMSIS flash algorithm layout: PrgCode, PrgData, DevDscr, linked at
 * 0. J-Link copies PrgCode and PrgData to the work RAM and calls the entries by symbol.
 */
SECTIONS
{
	PrgCode 0 : {
		*(.text .text.*)
		*(.rodata .rodata.*)
		*(.srodata .srodata.*)
	}

	PrgData : {
		*(.data .data.*)
		*(.sdata .sdata.*)
		*(.sbss .sbss.*)
		*(.bss .bss.*)
		*(COMMON)
	}

	DevDscr : {
		KEEP(*(DevDscr))
	}

	/DISCARD/ : { *(.comment .riscv.attributes .note.*) }
}
