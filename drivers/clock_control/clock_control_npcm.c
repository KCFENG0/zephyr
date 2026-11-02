/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_pcc

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_npcm.h>
#include <zephyr/dt-bindings/clock/npcm_clock.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_npcm, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

/* Driver config */
struct npcm_pcc_config {
	/* cdcg device base address */
	uintptr_t base_cdcg;
	/* pmc device base address */
	uintptr_t base_pmc;
};

struct freq_multiplier_t {
	uint32_t ofmclk;
	uint8_t hfcgn;
	uint8_t hfcgmh;
	uint8_t hfcgml;
};

#ifndef CONFIG_CLOCK_CONTROL_NPCM_FPGA
#ifdef CONFIG_SOC_SERIES_NPCM5
/*
 * NPCM5 HFCGN is XF_RANGE[7:6] | N[5:0], unlike the single XF_RANGE bit 7 on
 * NPCM4. XF_RANGE multiplies FMCLK by 1, 2, 4 or 8 to give OFMCLK.
 */
static struct freq_multiplier_t freq_multiplier[] = {
	{.ofmclk = 400000000, .hfcgn = 0xC2, .hfcgmh = 0x0B, .hfcgml = 0xEC},
	{.ofmclk = 384000000, .hfcgn = 0xC2, .hfcgmh = 0x0B, .hfcgml = 0x72},
	{.ofmclk = 200000000, .hfcgn = 0x82, .hfcgmh = 0x0B, .hfcgml = 0xEC},
	{.ofmclk = 192000000, .hfcgn = 0x82, .hfcgmh = 0x0B, .hfcgml = 0x72},
	{.ofmclk = 100000000, .hfcgn = 0x42, .hfcgmh = 0x0B, .hfcgml = 0xEC},
	{.ofmclk = 96000000, .hfcgn = 0x42, .hfcgmh = 0x0B, .hfcgml = 0x72},
	{.ofmclk = 80000000, .hfcgn = 0x42, .hfcgmh = 0x09, .hfcgml = 0x89},
	{.ofmclk = 66000000, .hfcgn = 0x42, .hfcgmh = 0x07, .hfcgml = 0xDE},
	{.ofmclk = 50000000, .hfcgn = 0x02, .hfcgmh = 0x0B, .hfcgml = 0xEC},
	{.ofmclk = 48000000, .hfcgn = 0x02, .hfcgmh = 0x0B, .hfcgml = 0x72},
	{.ofmclk = 40000000, .hfcgn = 0x02, .hfcgmh = 0x09, .hfcgml = 0x89},
	{.ofmclk = 33000000, .hfcgn = 0x02, .hfcgmh = 0x07, .hfcgml = 0xDE}};
#else
static struct freq_multiplier_t freq_multiplier[] = {
	{.ofmclk = 100000000, .hfcgn = 0x82, .hfcgmh = 0x0B, .hfcgml = 0xEC},
	{.ofmclk = 96000000, .hfcgn = 0x82, .hfcgmh = 0x0B, .hfcgml = 0x72},
	{.ofmclk = 80000000, .hfcgn = 0x82, .hfcgmh = 0x09, .hfcgml = 0x89},
	{.ofmclk = 66000000, .hfcgn = 0x82, .hfcgmh = 0x07, .hfcgml = 0xDE},
	{.ofmclk = 50000000, .hfcgn = 0x02, .hfcgmh = 0x0B, .hfcgml = 0xEC},
	{.ofmclk = 48000000, .hfcgn = 0x02, .hfcgmh = 0x0B, .hfcgml = 0x72},
	{.ofmclk = 40000000, .hfcgn = 0x02, .hfcgmh = 0x09, .hfcgml = 0x89},
	{.ofmclk = 33000000, .hfcgn = 0x02, .hfcgmh = 0x07, .hfcgml = 0xDE},
	{.ofmclk = 20000000, .hfcgn = 0x08, .hfcgmh = 0x13, .hfcgml = 0x13},
	{.ofmclk = 10000000, .hfcgn = 0x08, .hfcgmh = 0x09, .hfcgml = 0x89}};
#endif /* CONFIG_SOC_SERIES_NPCM5 */
#endif

/* PMC multi-registers */
#define NPCM_PWDWN_CTL_OFFSET(n) (((n) < 7) ? (0x07 + n) : (0x15 + (n - 7)))
#define NPCM_PWDWN_CTL(base, n)  (*(volatile uint8_t *)(base + NPCM_PWDWN_CTL_OFFSET(n)))
#define NPCM_CLOCK_REG_OFFSET(n)                                                                   \
	(((n) >> NPCM_CLOCK_PWDWN_GROUP_OFFSET) & NPCM_CLOCK_PWDWN_GROUP_MASK)
#define NPCM_CLOCK_REG_BIT_OFFSET(n)                                                               \
	(((n) >> NPCM_CLOCK_PWDWN_BIT_OFFSET) & NPCM_CLOCK_PWDWN_BIT_MASK)

#define DRV_CONFIG(dev) ((const struct npcm_pcc_config *)(dev)->config)

static inline int npcm_clock_control_on(const struct device *dev, clock_control_subsys_t sub_system)
{
	uint32_t clk_cfg = (uint32_t)sub_system;
	const uint32_t pmc_base = DRV_CONFIG(dev)->base_pmc;

	if ((clk_cfg & BIT(NPCM_CLOCK_PWDWN_VALID_OFFSET)) == 0) {
		LOG_ERR("Unsupported clock cfg %d", clk_cfg);
		return -EINVAL;
	}

	/* Clear related PD (Power-Down) bit of module to turn on clock */
	NPCM_PWDWN_CTL(pmc_base, NPCM_CLOCK_REG_OFFSET(clk_cfg)) &=
		~(BIT(NPCM_CLOCK_REG_BIT_OFFSET(clk_cfg)));
	return 0;
}

static inline int npcm_clock_control_off(const struct device *dev,
					 clock_control_subsys_t sub_system)
{
	uint32_t clk_cfg = (uint32_t)sub_system;
	const uint32_t pmc_base = DRV_CONFIG(dev)->base_pmc;

	if ((clk_cfg & BIT(NPCM_CLOCK_PWDWN_VALID_OFFSET)) == 0) {
		LOG_ERR("Unsupported clock cfg %d", clk_cfg);
		return -EINVAL;
	}

	/* Set related PD (Power-Down) bit of module to turn off clock */
	NPCM_PWDWN_CTL(pmc_base, NPCM_CLOCK_REG_OFFSET(clk_cfg)) |=
		BIT(NPCM_CLOCK_REG_BIT_OFFSET(clk_cfg));
	return 0;
}

static int npcm_clock_control_get_subsys_rate(const struct device *dev,
					      clock_control_subsys_t sub_system, uint32_t *rate)
{
	ARG_UNUSED(dev);
	uint32_t clk_cfg = (uint32_t)sub_system;
	uint8_t bus_id = ((clk_cfg >> NPCM_CLOCK_BUS_OFFSET) & NPCM_CLOCK_BUS_MASK);

	switch (bus_id) {
	case NPCM_CLOCK_BUS_APB1:
		*rate = NPCM_APB_CLOCK(1);
		break;
	case NPCM_CLOCK_BUS_APB2:
		*rate = NPCM_APB_CLOCK(2);
		break;
	case NPCM_CLOCK_BUS_APB3:
		*rate = NPCM_APB_CLOCK(3);
		break;
	case NPCM_CLOCK_BUS_AHB6:
		*rate = CORE_CLK / (AHB6DIV_VAL + 1);
		break;
	case NPCM_CLOCK_BUS_FIU:
		*rate = CORE_CLK / (FIUDIV_VAL + 1);
		break;
	case NPCM_CLOCK_BUS_I3C_TC:
		*rate = NPCM_APB_CLOCK(3) / (I3C_TC_DIV_VAL + 1);
		break;
	case NPCM_CLOCK_BUS_CORE:
		*rate = CORE_CLK;
		break;
	case NPCM_CLOCK_BUS_OSC:
		*rate = OFMCLK;
		break;
	case NPCM_CLOCK_BUS_LFCLK:
		*rate = LFCLK;
		break;
	case NPCM_CLOCK_BUS_FMCLK:
		*rate = FMCLK;
		break;
	case NPCM_CLOCK_BUS_USB20_CLK:
		*rate = USB20_CLK;
		break;
	case NPCM_CLOCK_BUS_SIO_CLK:
		*rate = SIO_CLK;
		break;
	default:
		*rate = 0U;
		/* Invalid parameters */
		return -EINVAL;
	}

	return 0;
}

/* Clock controller driver registration */
static struct clock_control_driver_api npcm_clock_control_api = {
	.on = npcm_clock_control_on,
	.off = npcm_clock_control_off,
	.get_rate = npcm_clock_control_get_subsys_rate,
};

#if !defined(CONFIG_CLOCK_CONTROL_NPCM_FPGA) && defined(CONFIG_SOC_SERIES_NPCM5)
/* NPCM5 clock limits */
BUILD_ASSERT(CORE_CLK <= MHZ(200), "PERIPH_CLK exceeds 200 MHz, raise core-prescaler");
BUILD_ASSERT(CORE_CLK / (AHB6DIV_VAL + 1) <= MHZ(200), "AHB6_CLK exceeds 200 MHz");
BUILD_ASSERT(CORE_CLK / (FIUDIV_VAL + 1) <= MHZ(100), "FIUCLK exceeds 100 MHz");
BUILD_ASSERT(NPCM_APB_CLOCK(1) <= MHZ(100), "APB1_CLK exceeds 100 MHz");
BUILD_ASSERT(NPCM_APB_CLOCK(2) <= MHZ(100), "APB2_CLK exceeds 100 MHz");
BUILD_ASSERT(NPCM_APB_CLOCK(3) <= MHZ(100), "APB3_CLK exceeds 100 MHz");
BUILD_ASSERT(MCLK / (M7DIV_VAL + 1) <= MHZ(400), "M7_FCLK exceeds 400 MHz");
BUILD_ASSERT(MCLK / (MRAMDIV_VAL + 1) <= MIN(MCLK / 2, MHZ(200)),
	     "MRAM_CLK exceeds MCLK / 2 or 200 MHz");
BUILD_ASSERT(OFMCLK / (CSSDIV_VAL + 1) <= MHZ(50), "CSS_FCLK exceeds 50 MHz");
#endif

#ifndef CONFIG_CLOCK_CONTROL_NPCM_FPGA
/* Set all clock prescalers of core and peripherals. */
static void npcm_hfcg_set_dividers(struct cdcg_reg *priv)
{
#ifdef CONFIG_SOC_SERIES_NPCM5
	priv->hfcgp = (FPRED_VAL << 4) | (I3C_TC_DIV_VAL << 2) | AHB6DIV_VAL;
	priv->hfcbcd = APB1DIV_VAL | (APB2DIV_VAL << 4);
	priv->hfcbcd1 = (FIUDIV_VAL << 4) | APB3DIV_VAL;
	priv->hfcbcd2 = (CSSDIV_VAL << 4) | (MRAMDIV_VAL << 2) | M7DIV_VAL;
#else
	priv->hfcgp = (FPRED_VAL << 4) | AHB6DIV_VAL;
	priv->hfcbcd = APB1DIV_VAL | (APB2DIV_VAL << 4);
	priv->hfcbcd1 = (I3C_TC_DIV_VAL << 2) | FIUDIV_VAL;
	priv->hfcbcd2 = APB3DIV_VAL;
#endif

	/*
	 * Let the new dividers take effect before M/N may change: at least 16
	 * PERIPH_CLK cycles. Each CDCG read is an APB1 access, and APB1_CLK is
	 * never faster than PERIPH_CLK.
	 */
	for (int i = 0; i < 16; i++) {
		(void)priv->hfcgp;
	}
}

/* OFMCLK the frequency multiplier runs at now, or 0 if its M/N is not in the table */
static uint32_t npcm_hfcg_cur_ofmclk(const struct cdcg_reg *priv)
{
	for (int i = 0; i < ARRAY_SIZE(freq_multiplier); i++) {
		const struct freq_multiplier_t *f = &freq_multiplier[i];

		if (priv->hfcgn == f->hfcgn && priv->hfcgml == f->hfcgml &&
		    priv->hfcgmh == f->hfcgmh) {
			return f->ofmclk;
		}
	}

	return 0;
}
#endif /* CONFIG_CLOCK_CONTROL_NPCM_FPGA */

static int npcm_clock_control_init(const struct device *dev)
{
#ifdef CONFIG_CLOCK_CONTROL_NPCM_FPGA
	/* FPGA mode: bypass all clock configuration, use fixed clocks */
	ARG_UNUSED(dev);
	return 0;
#else
	struct cdcg_reg *const priv = (struct cdcg_reg *)(DRV_CONFIG(dev)->base_cdcg);
	struct freq_multiplier_t *freq_p = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(freq_multiplier); i++) {
		if (freq_multiplier[i].ofmclk == OFMCLK) {
			freq_p = &freq_multiplier[i];
			break;
		}
	}

	if (freq_p == NULL) {
		LOG_ERR("Unsupported OFMCLK frequency %d", OFMCLK);
		return -EINVAL;
	}

	/*
	 * Slow the dividers down before raising OFMCLK so that no derived clock
	 * exceeds its limit while the multiplier changes. An unknown M/N counts
	 * as a raise.
	 */
	if (freq_p->ofmclk > npcm_hfcg_cur_ofmclk(priv)) {
		npcm_hfcg_set_dividers(priv);
	}

	/*
	 * Resetting the OFMCLK (even to the same value) will make the clock
	 * unstable for a little which can affect peripheral communication like
	 * eSPI. Skip this if not needed.
	 */
	if (priv->hfcgn != freq_p->hfcgn || priv->hfcgml != freq_p->hfcgml ||
	    priv->hfcgmh != freq_p->hfcgmh) {
		/*
		 * Configure frequency multiplier M/N values according to
		 * the requested OFMCLK (Unit:Hz).
		 */
		priv->hfcgn = freq_p->hfcgn;
		priv->hfcgml = freq_p->hfcgml;
		priv->hfcgmh = freq_p->hfcgmh;

		/* Load M and N values into the frequency multiplier */
		priv->hfcgctrl |= BIT(NPCM_HFCGCTRL_LOAD);
		/* Wait for stable */
		while (priv->hfcgctrl & BIT(NPCM_HFCGCTRL_CLK_CHNG)) {
		}
	}

	npcm_hfcg_set_dividers(priv);

	return 0;
#endif /* CONFIG_CLOCK_CONTROL_NPCM_FPGA */
}

const struct npcm_pcc_config pcc_config = {
	.base_cdcg = DT_INST_REG_ADDR_BY_NAME(0, cdcg),
	.base_pmc = DT_INST_REG_ADDR_BY_NAME(0, pmc),
};

DEVICE_DT_INST_DEFINE(0, &npcm_clock_control_init, NULL, NULL, &pcc_config, PRE_KERNEL_1,
		      CONFIG_KERNEL_INIT_PRIORITY_OBJECTS, &npcm_clock_control_api);
