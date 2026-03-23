/*
 * Place .rom_hooks in the data region (before bss/noinit) so that
 * objcopy -O binary does not need to fill the NOBITS gap with zeros.
 */
SECTION_PROLOGUE(.rom_hooks,, SUBALIGN(4))
{
	KEEP(*(.rom_hooks_data))
	KEEP(*(.rom_hooks_fn))
} GROUP_LINK_IN(ROMABLE_REGION)
