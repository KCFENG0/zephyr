/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CLOCK_CONTROL_NPCM_H_
#define CLOCK_CONTROL_NPCM_H_

/*
 * Core Domain Clock Generator (CDCG) device registers
 */
struct cdcg_reg {
	/* High Frequency Clock Generator (HFCG) registers */
	/* 0x000: HFCG Control */
	volatile uint8_t hfcgctrl;
	volatile uint8_t reserved1;
	/* 0x002: HFCG M Low Byte Value */
	volatile uint8_t hfcgml;
	volatile uint8_t reserved2;
	/* 0x004: HFCG M High Byte Value */
	volatile uint8_t hfcgmh;
	volatile uint8_t reserved3;
	/* 0x006: HFCG N Value */
	volatile uint8_t hfcgn;
	volatile uint8_t reserved4;
	/* 0x008: HFCG Prescaler */
	volatile uint8_t hfcgp;
	volatile uint8_t reserved5[7];
	/* 0x010: HFCG Bus Clock Dividers */
	volatile uint8_t hfcbcd;
	volatile uint8_t reserved6;
	/* 0x012: HFCG Bus Clock Dividers */
	volatile uint8_t hfcbcd1;
	volatile uint8_t reserved7;
	/* 0x014: HFCG Bus Clock Dividers */
	volatile uint8_t hfcbcd2;
	volatile uint8_t reserved12[3];
	/* 0x018: HFCG Bus Clock Dividers */
	volatile uint8_t hfcbcd4;
	volatile uint8_t reserved13[4];
	/* 0x01d: HFCG Bus Clock Dividers */
	volatile uint8_t hfcbcd3;
};

/* CDCG register fields */
#define NPCM_HFCGCTRL_LOAD     0
#define NPCM_HFCGCTRL_LOCK     2
#define NPCM_HFCGCTRL_CLK_CHNG 7

/* Clock settings from pcc node */
#define NPCM_NODE_PCC_LABEL DT_NODELABEL(pcc)
/* Target OFMCLK freq */
#define OFMCLK      DT_PROP(NPCM_NODE_PCC_LABEL, clock_frequency)
/* Core clock prescaler */
#define FPRED_VAL   (DT_PROP(NPCM_NODE_PCC_LABEL, core_prescaler) - 1)
/* APB1 clock divider */
#define APB1DIV_VAL (DT_PROP(NPCM_NODE_PCC_LABEL, apb1_prescaler) - 1)
/* APB2 clock divider */
#define APB2DIV_VAL (DT_PROP(NPCM_NODE_PCC_LABEL, apb2_prescaler) - 1)
/* APB3 clock divider */
#define APB3DIV_VAL (DT_PROP(NPCM_NODE_PCC_LABEL, apb3_prescaler) - 1)
/* AHB6 clock divider*/
#define AHB6DIV_VAL (DT_PROP(NPCM_NODE_PCC_LABEL, ahb6_prescaler) - 1)
/* FIU clock divider */
#define FIUDIV_VAL  (DT_PROP(NPCM_NODE_PCC_LABEL, fiu_prescaler) - 1)
/* FIU1/FIU2 clock dividers, default to the FIU divider */
#define FIU1DIV_VAL (DT_PROP_OR(NPCM_NODE_PCC_LABEL, fiu1_prescaler, \
				DT_PROP(NPCM_NODE_PCC_LABEL, fiu_prescaler)) - 1)
#define FIU2DIV_VAL (DT_PROP_OR(NPCM_NODE_PCC_LABEL, fiu2_prescaler, \
				DT_PROP(NPCM_NODE_PCC_LABEL, fiu_prescaler)) - 1)
/* I3C clock divider */
#define I3C_TC_DIV_VAL  (DT_PROP(NPCM_NODE_PCC_LABEL, i3c_tc_prescaler) - 1)
/* M7 fast clock divider (NPCM5) */
#define M7DIV_VAL   (DT_PROP(NPCM_NODE_PCC_LABEL, m7_prescaler) - 1)
/* MRAM clock divider (NPCM5) */
#define MRAMDIV_VAL (DT_PROP(NPCM_NODE_PCC_LABEL, mram_prescaler) - 1)
/* Caliptra Sub-System fast clock divider (NPCM5) */
#define CSSDIV_VAL  (DT_PROP(NPCM_NODE_PCC_LABEL, css_prescaler) - 1)

/* Core domain clock */
#define CORE_CLK           (OFMCLK / DT_PROP(NPCM_NODE_PCC_LABEL, core_prescaler))
/* Low Frequency clock */
#define LFCLK              32768
/* FMUL clock: XF_RANGE divides OFMCLK by 8, 4 or 2 above 200, 100 and 50 MHz */
#if (OFMCLK > MHZ(200))
#define FMCLK              (OFMCLK / 8)
#elif (OFMCLK > MHZ(100))
#define FMCLK              (OFMCLK / 4)
#elif (OFMCLK > MHZ(50))
#define FMCLK              (OFMCLK / 2) /* FMUL clock = OFMCLK/2 */
#else
#define FMCLK              OFMCLK /* FMUL clock = OFMCLK */
#endif
/* MCLK clock */
#define MCLK               OFMCLK /* MCLK clock = OFMCLK */
/* APBs source clock: PERIPH_CLK (CORE_CLK) on NPCM5, MCLK on NPCM4 */
#ifdef CONFIG_SOC_SERIES_NPCM5
#define APBSRC_CLK         CORE_CLK
#else
#define APBSRC_CLK         MCLK
#endif
/* USB2.0 clock */
#define USB20_CLK          MHZ(12)
/* SIO clock */
#define SIO_CLK            MHZ(48)
/* Get APB clock freq */
#define NPCM_APB_CLOCK(no) (APBSRC_CLK / (APB##no##DIV_VAL + 1))

#endif /* CLOCK_CONTROL_NPCM_H_ */
