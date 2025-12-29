/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NUVOTON_NPCM5_REG_DEF_ADC_H
#define _NUVOTON_NPCM5_REG_DEF_ADC_H

/*
 * NPCM5 SAR ADC Register Structure (Compatible with NPCX ADC)
 * 
 * Note: The NPCM family has two ADC types:
 * - DSADC (struct adc_reg in reg_def.h) - Delta-Sigma ADC with 24 channels
 * - SAR ADC (this structure) - SAR ADC with 16 channels compatible with NPCX
 * 
 * This header overrides struct adc_reg from reg_def.h when CONFIG_ADC_NPCX is enabled.
 * It must be included AFTER reg/reg_def.h
 */

#if defined(CONFIG_ADC_NPCM_SARADC)

/* Redefine struct adc_reg for SAR ADC (NPCX compatible) */
#define adc_reg adc_reg_dsadc  /* Rename old DSADC structure to avoid conflict */
 
/* Now define the NPCX-compatible SAR ADC structure for NPCM SARADC driver */
struct adc_reg {
	/* 0x000: ADC Status */
	volatile uint16_t ADCSTS;
	/* 0x002: ADC Configuration */
	volatile uint16_t ADCCNF;
	/* 0x004: ADC Timing Control */
	volatile uint16_t ATCTL;
	/* 0x006: ADC Single Channel Address */
	volatile uint16_t ASCADD;
	/* 0x008: ADC Scan Channels Select */
	volatile uint16_t ADCCS;
	/* 0x00A: ADC Scan Channels Select 2 */
	volatile uint16_t ADCCS2;
	volatile uint8_t reserved1[14];
	/* 0x01A:  Threshold Status */
	volatile uint16_t THRCTS;
	volatile uint8_t reserved2[4];
	/* 0x020: Internal register 1 for ADC Speed */
	volatile uint16_t ADCCNF2;
	/* 0x022: Internal register 2 for ADC Speed */
	volatile uint16_t GENDLY;
	volatile uint8_t reserved3[2];
	/* 0x026: Internal register 3 for ADC Speed */
	volatile uint16_t MEAST;
};

/* ADC internal inline functions for multi-registers */
static inline uint32_t npcx_chndat_offset(uint32_t ch)
{
	return (0x40 + (ch * 2));
}

static inline uint32_t npcx_thrctl_offset(uint32_t ctrl)
{
	return (0x30 + (ctrl * 2));
}

#define NPCX_CHNDAT_OFFSET(ch) npcx_chndat_offset(ch)
#define NPCX_THRCTL_OFFSET(ctrl) npcx_thrctl_offset(ctrl)

#define CHNDAT(base, ch) \
	(*(volatile uint16_t *)((base) + NPCX_CHNDAT_OFFSET(ch)))
#define THRCTL(base, ctrl) \
	(*(volatile uint16_t *)(base + NPCX_THRCTL_OFFSET(ctrl)))

/* Threshold Deassertion Control offset function */
static inline uint32_t npcx_thr_dctl_offset(uint32_t ctrl)
{
	return (0x38 + (ctrl * 2));
}

#define NPCX_THR_DCTL_OFFSET(ctrl) npcx_thr_dctl_offset(ctrl)
#define THR_DCTL(base, ctrl) \
	(*(volatile uint16_t *)((base) + NPCX_THR_DCTL_OFFSET(ctrl)))

/* ADC register fields */
#define NPCX_ATCTL_SCLKDIV_FIELD              FIELD(0, 6)
#define NPCX_ATCTL_DLY_FIELD                  FIELD(8, 3)
#define NPCX_ASCADD_SADDR_FIELD               FIELD(0, 5)
#define NPCX_ADCSTS_EOCEV                     0
#define NPCX_ADCSTS_EOCCEV                    1
#define NPCX_ADCSTS_EOTCEV                    2
#define NPCX_ADCSTS_OVFEV                     3
#define NPCX_ADCSTS_TEOCCEV                   5
#define NPCX_ADCCNF_ADCEN                     0
#define NPCX_ADCCNF_ADCMD_FIELD               FIELD(1, 2)
#define NPCX_ADCCNF_ADCRPTC                   3
#define NPCX_ADCCNF_START                     4
#define NPCX_ADCCNF_ADCTTE                    5
#define NPCX_ADCCNF_INTECEN                   6
#define NPCX_ADCCNF_INTECCEN                  7
#define NPCX_ADCCNF_INTETCEN                  8
#define NPCX_ADCCNF_INTOVFEN                  9
#define NPCX_ADCCNF_STOP                      11
#define NPCX_CHNDAT_CHDAT_FIELD               FIELD(0, 10)
#define NPCX_CHNDAT_NEW                       15
#define NPCX_THRCTS_ADC_WKEN                  15
#define NPCX_THRCTS_THR3_IEN                  10
#define NPCX_THRCTS_THR2_IEN                  9
#define NPCX_THRCTS_THR1_IEN                  8
#define NPCX_THRCTS_ADC_EVENT                 7
#define NPCX_THRCTS_THR3_STS                  2
#define NPCX_THRCTS_THR2_STS                  1
#define NPCX_THRCTS_THR1_STS                  0
#define NPCX_THR_DCTL_THRD_EN                 15
#define NPCX_THR_DCTL_THR_DVAL                FIELD(0, 10)

/* Speed register default values from SARADC spec */
#define NPCX_ADCCNF2_DEFAULT_VAL              0x8901  /* Speed 1 Register default */
#define NPCX_GENDLY_DEFAULT_VAL               0x0100  /* Speed 2 Register default */
#define NPCX_MEAST_DEFAULT_VAL                0x0405  /* Speed 3 Register default */

/* V2T (Voltage to Temperature) related definitions */
#ifdef CONFIG_ADC_V2T_NPCX
/* V2T Control register offset */
#define NPCX_V2T_CTRL_OFFSET                  0x70
#define V2T_CTRL(base) (*(volatile uint16_t *)((base) + NPCX_V2T_CTRL_OFFSET))

/* V2T Control register fields */
#define NPCX_V2T_CTRL_TEMP_EN                 0
#define NPCX_V2T_CTRL_OVT_EN                  1
#define NPCX_V2T_CTRL_TINTCC_EN               3
#define NPCX_V2T_CTRL_OVT_MODE                6

/* Temperature channel data offset */
#define NPCX_TEMP_CH_OFFSET                   0x74
#define TEMP_CH(base) (*(volatile uint16_t *)((base) + NPCX_TEMP_CH_OFFSET))

/* Temperature channel data registers */
static inline uint32_t npcx_tchndat_offset(uint32_t ch)
{
	return (0x78 + (ch * 2));
}

#define NPCX_TCHNDAT_OFFSET(ch) npcx_tchndat_offset(ch)
#define TCHNDAT(base, ch) \
	(*(volatile uint16_t *)((base) + NPCX_TCHNDAT_OFFSET(ch)))
#endif /* CONFIG_ADC_V2T_NPCX */

/* Threshold comparator V2 support */
#ifdef CONFIG_ADC_NPCX_CMP_V2
#define NPCX_THEN_OFFSET                      0x2C
#define THEN(base) (*(volatile uint16_t *)((base) + NPCX_THEN_OFFSET))
#endif

#endif /* CONFIG_ADC_NPCM_SARADC */

#endif /* _NUVOTON_NPCM5_REG_DEF_ADC_H */
