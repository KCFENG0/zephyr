/*
 * Place .rom_hooks in the data region (before bss/noinit) so that
 * objcopy -O binary does not need to fill the NOBITS gap with zeros.
 *
 * Non-XIP only — included via zephyr_linker_sources(DATA_SECTIONS).
 * For XIP builds, the identical section is defined in linker.ld instead
 * (appended after the main linker script) to avoid corrupting
 * __data_region_end.  Keep both definitions in sync.
 */
SECTION_PROLOGUE(.rom_hooks,, SUBALIGN(4))
{
	KEEP(*(.rom_hooks_data))
	KEEP(*(.rom_hooks_fn))
} GROUP_LINK_IN(ROMABLE_REGION)
