/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/cache.h>
#include <zephyr/linker/sections.h>
#include <cmsis_core.h>
#include <soc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

#define NPCM_CORST_CTL_MASK					\
	(BIT(NPCM_CORST_CTL_CM7) | BIT(NPCM_CORST_CTL_EMAC) |	\
	 BIT(NPCM_CORST_CTL_DMA) | BIT(NPCM_CORST_CTL_USBH) |	\
	 BIT(NPCM_CORST_CTL_USBD) | BIT(NPCM_CORST_CTL_SIOX1) |	\
	 BIT(NPCM_CORST_CTL_SIOX2) | BIT(NPCM_CORST_CTL_SPER))

static inline void npcm_route_corst_reset(struct scfg_reg *inst_scfg)
{
	inst_scfg->CORST_CTL = (inst_scfg->CORST_CTL & ~NPCM_CORST_CTL_MASK) |
			       NPCM_CORST_CTL_MASK;
}

#ifdef CONFIG_SOC_RESET_HOOK
void soc_reset_hook(void)
{
	/*
	 * Cache enable moved to npcm5_cache_init() (PRE_KERNEL_1, prio 0)
	 * so it runs after z_arm_mpu_init() has programmed Device/nocache
	 * MPU regions. Enabling D-cache before MPU is configured risks
	 * caching lines from regions whose attributes are about to change.
	 */
	uintptr_t scfg_base = SCFG_BASE_ADDR;
	struct scfg_reg *inst_scfg = (struct scfg_reg *)scfg_base;

	/* RMW only bit 0; other DEVALT10 bits may be programmed by the ROM
	 * (e.g. SPI quad-mode straps) and a plain byte write would clear them.
	 */
	uint8_t devalt10 = inst_scfg->DEVALT10;

	devalt10 &= ~BIT(NPCM_DEVALT10_CRGPIO_SELECT_SL);
#ifdef CONFIG_GPIO_NPCM_RESET_SL_POWER_UP
	devalt10 |= BIT(NPCM_DEVALT10_CRGPIO_SELECT_SL);
#endif
	inst_scfg->DEVALT10 = devalt10;

	/*
	 * Route the CORST pin reset to the full NPCM500 domain set: CM7,
	 * EMAC, DMA, USBH, USBD, SIOX1, SIOX2 and SPER. CORST_CTL powers up as
	 * 0 and is only reset by VSB power-up, so clear and reassert the full
	 * mask to avoid stale ROM/boot routing surviving into the next reset.
	 */
	npcm_route_corst_reset(inst_scfg);

	/*
	 * Route the TWD watchdog reset to the Cortex-M7 core. WDTRST_CTL powers
	 * up as 0 (VSB reset only), so set it here instead of in the watchdog
	 * driver, which is shared with NPCM4 where this register does not exist.
	 */
	inst_scfg->WDTRST_CTL |= BIT(NPCM_WDTRST_CTL_CM7);
}
#endif

/*
 * Enable I-cache and D-cache after MPU regions are programmed.
 *
 * Boot order on Cortex-M:
 *   soc_reset_hook -> z_arm_init_arch_hw_at_boot -> z_cstart
 *     -> arch_kernel_init() [runs z_arm_mpu_init + static MPU regions]
 *     -> SYS_INIT(PRE_KERNEL_1) chain  <-- this function runs here
 *
 * Calls CMSIS SCB_* directly (not sys_cache_*) so caches come up
 * regardless of CONFIG_CACHE_MANAGEMENT. Runtime DMA flush/invalidate
 * in drivers should still go through the Zephyr sys_cache_* API.
 */
static int npcm5_cache_init(void)
{
	/*
	 * SCB_Enable{I,D}Cache() invalidates the cache as part of the
	 * enable sequence (only when CCR.{IC,DC} is 0). For a warm reset
	 * where CCR survives, z_arm_init_arch_hw_at_boot() has already
	 * disabled and invalidated caches before we get here.
	 */
#if defined(CONFIG_ICACHE)
	SCB_EnableICache();
#endif
#if defined(CONFIG_DCACHE)
	SCB_EnableDCache();
#endif

	return 0;
}
SYS_INIT(npcm5_cache_init, PRE_KERNEL_1, 0);

#if CONFIG_SOC_LATE_INIT_HOOK
__pinned_bss bool npcm_sys_application;

/* Hook at the end of the kernel initialization */
void soc_late_init_hook(void)
{
	/* Set the flag to indicate the system is in application stage */
	npcm_sys_application = true;
}
#endif
