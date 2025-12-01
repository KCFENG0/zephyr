/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ADC_NPCM_SAR_H_
#define ZEPHYR_DRIVERS_ADC_NPCM_SAR_H_

#include <zephyr/types.h>

/* NPCM SAR ADC Register Definitions */

/* NPCM SAR ADC Control Register (ADCCON) */
#define NPCM_ADCCON_ADCST		    BIT(0)		    /* ADC Start */
#define NPCM_ADCCON_ADCEN		    BIT(1)		    /* ADC Enable */
#define NPCM_ADCCON_ADCINTEN	    BIT(2)		    /* ADC Interrupt Enable */
#define NPCM_ADCCON_REFSEL		    BIT(3)		    /* Reference Selection */

/* NPCM SAR ADC Status Register (ADCSTS) */
#define NPCM_ADCSTS_EOAINTS		    BIT(0)		    /* End of ADC Interrupt Status */
#define NPCM_ADCSTS_TH1INTS		    BIT(1)		    /* Threshold 1 Interrupt Status */
#define NPCM_ADCSTS_TH2INTS		    BIT(2)		    /* Threshold 2 Interrupt Status */
#define NPCM_ADCSTS_TH3INTS		    BIT(3)		    /* Threshold 3 Interrupt Status */

/* NPCM SAR ADC Data Register (ADCDAT) */
#define NPCM_ADCDAT_ADCDATA_FIELD	GENMASK(9, 0)	/* ADC Data Field (10-bit) */

/* NPCM SAR ADC Timing Control Register (ATCTL) */
#define NPCM_ATCTL_SCLKDIV_FIELD	GENMASK(5, 0)	/* Sample Clock Divider */
#define NPCM_ATCTL_DLY_FIELD		GENMASK(11, 8)	/* Delay Field */

/* NPCM SAR ADC Clock Definitions */
#define NPCM_ADC_CLK			    2000000		    /* Target ADC clock: 2MHz */

/* Maximum number of thresholds supported */
#define NPCM_ADC_MAX_THRESHOLDS		3

/* NPCM SAR ADC Register Structure */
struct npcm_adc_regs {
    volatile uint32_t adccon;		/* ADC Control Register */
    volatile uint32_t adcsts;		/* ADC Status Register */
    volatile uint32_t adcdat;		/* ADC Data Register */
    volatile uint32_t atctl;		/* ADC Timing Control Register */
    volatile uint32_t adcchansel;	/* ADC Channel Selection Register */
    volatile uint32_t adcth1;		/* ADC Threshold 1 Register */
    volatile uint32_t adcth2;		/* ADC Threshold 2 Register */
    volatile uint32_t adcth3;		/* ADC Threshold 3 Register */
};

#endif /* ZEPHYR_DRIVERS_ADC_NPCM_SAR_H_ */
