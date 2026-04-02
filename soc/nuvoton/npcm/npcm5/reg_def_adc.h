/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NUVOTON_NPCM5_REG_DEF_ADC_H
#define _NUVOTON_NPCM5_REG_DEF_ADC_H

/*
 * NPCM5 SAR ADC Register Structure
 *
 * Note: The NPCM family has two ADC types:
 * - DSADC (struct adc_reg in reg_def.h) - Delta-Sigma ADC
 * - SAR ADC (this structure) - 10-bit SAR ADC with 14 channels (ADC0-ADC13)
 *   per datasheet section 4.15 (14-1 analog multiplexer, 8 voltage inputs +
 *   6 power pins)
 *
 * Register layout (datasheet section 4.15.8 / base address 0x400D_1000):
 *   0x00: ADCSTS   - ADC Status
 *   0x02: ADCCNF   - ADC Configuration
 *   0x04: ATCTL    - ADC Timing Control
 *   0x06: ASCADD   - ADC Single Channel Address (SADDR[4:0], TTADDR[12:8])
 *   0x08: ADCCS    - ADC Scan Channels Select (CC0-CC13, 14 channels)
 *   0x14: THRCTL1  - Threshold Control 1 (THEN[15], L_H[14], CHNSEL[13:10], THRVAL[9:0])
 *   0x16: THRCTL2  - Threshold Control 2
 *   0x18: THRCTL3  - Threshold Control 3
 *   0x1A: THRCTS   - Threshold Status (THRx_IEN[10:8], ADC_EVENT[7], THRx_STS[2:0])
 *   0x20: ADCCNF2  - Speed Register 1 (default 0x8901)
 *   0x22: GENDLY   - Speed Register 2 (default 0x0100)
 *   0x26: MEAST    - Speed Register 3 (default 0x0405)
 *   0x3A: THR_DCTL1 - Deassertion Threshold Control 1
 *   0x3C: THR_DCTL2 - Deassertion Threshold Control 2
 *   0x3E: THR_DCTL3 - Deassertion Threshold Control 3
 *   0x40-0x5A: CHN0DAT-CHN13DAT - Channel Data Buffers (CHDAT[9:0], NEW[15])
 *   V2T / Extended registers (same base):
 *   0x100: V2T_CTRL    - ADC Voltage-to-Temperature Control
 *   0x102: TEMP_CH     - Temperature Channel Enable
 *   0x106: OVT_STS     - Over-Temperature Status (W1C)
 *   0x10E-0x11C: TCHN0DAT-TCHN7DAT  - Temperature Channel Data Buffers (8 ch: VIN1-VIN16)
 *                 Each: NEW[15], TCHDAT[10:3] (integer), TCHDAT_Fraction[2:0]
 *   0x12E-0x150: THR_O_0-THR_O_16   - Thermistor Offset table (17 entries, THR_OFFSET[13:0])
 *   0x152-0x172: THR_S_0-THR_S_16   - Thermistor Slope table (17 entries, THR_SLOPE[9:0])
 *   0x180-0x196: TEMP_THRCTL0-11    - Temperature Channel Threshold Control (12 entries)
 *                 Each: TEMP_THEN[15], TEMP_THRVAL[10:3], TEMP_THRVAL_Fraction[2:0]
 *   0x1A0-0x1B6: TEMP_THR_DCTL0-11 - Temperature Channel Deassertion Threshold (12 entries)
 *                 Each: TEMP_THRD_EN[15], TEMP_THR_DVAL[10:3], TEMP_THR_DVAL_Fraction[2:0]
 *   0x1C0-0x1CA: ADCTD1-0PO-ADCTD11-10PO - Temperature Post-Offset adjust (6 pair registers)
 *                 Each: TDn+1_POST_OFFSET[15:8], TDn_POST_OFFSET[7:0]
 *
 * This header overrides struct adc_reg from reg_def.h when CONFIG_ADC_NPCM_SARADC is enabled.
 * It must be included AFTER reg/reg_def.h
 */

#if defined(CONFIG_ADC_NPCM_SARADC)

/* Redefine struct adc_reg for SAR ADC (rename DSADC struct to avoid conflict) */
#define adc_reg adc_reg_dsadc  /* Rename old DSADC structure */

/* NPCM5 SAR ADC register structure.
 * Base address: 0x400D_1000 (saradc@400d1000 in DT)
 * Timer-triggered mode uses MFT16-1 TA2 output (see datasheet section 4.15.2).
 */
struct adc_reg {
	/* 0x000: ADC Status */
	volatile uint16_t ADCSTS;
	/* 0x002: ADC Configuration */
	volatile uint16_t ADCCNF;
	/* 0x004: ADC Timing Control */
	volatile uint16_t ATCTL;
	/* 0x006: ADC Single Channel Address */
	volatile uint16_t ASCADD;
	/* 0x008: ADC Scan Channels Select (CC13:CC0 at bits[13:0]; bits[15:14] reserved) */
	volatile uint16_t ADCCS;
	/* 0x00A-0x019: Reserved (no registers between ADCCS and THRCTS per datasheet) */
	volatile uint8_t reserved1[16];
	/* 0x01A: Threshold Status */
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
	/* Datasheet: THRCTL1=0x14h, THRCTL2=0x16h, THRCTL3=0x18h */
	return (0x14 + (ctrl * 2));
}

#define NPCX_CHNDAT_OFFSET(ch) npcx_chndat_offset(ch)
#define NPCX_THRCTL_OFFSET(ctrl) npcx_thrctl_offset(ctrl)

#define CHNDAT(base, ch) \
	(*(volatile uint16_t *)((base) + NPCX_CHNDAT_OFFSET(ch)))
#define THRCTL(base, ctrl) \
	(*(volatile uint16_t *)((base) + NPCX_THRCTL_OFFSET(ctrl)))

/* Threshold Deassertion Control offset function */
static inline uint32_t npcx_thr_dctl_offset(uint32_t ctrl)
{
	/* Datasheet register map: THR_DCTL1=0x3Ah, THR_DCTL2=0x3Ch, THR_DCTL3=0x3Eh */
	return (0x3A + (ctrl * 2));
}

#define NPCX_THR_DCTL_OFFSET(ctrl) npcx_thr_dctl_offset(ctrl)
#define THR_DCTL(base, ctrl) \
	(*(volatile uint16_t *)((base) + NPCX_THR_DCTL_OFFSET(ctrl)))

/* ADC register fields */
#define NPCX_ATCTL_SCLKDIV_FIELD              FIELD(0, 6)   /* Clock div factor [5:0], max 64 */
#define NPCX_ATCTL_DLY_FIELD                  FIELD(8, 3)   /* Channel delay [10:8], set to 010 */
#define NPCX_ASCADD_SADDR_FIELD               FIELD(0, 5)   /* Software-Triggered Address [4:0] */
#define NPCX_ASCADD_TTADDR_FIELD              FIELD(8, 5)   /* Timer-Triggered Address [12:8] */
/* ADCSTS event flag bits (R/W1C) - datasheet section 4.15.9 offset 0x00:
 *   Voltage events (bits[3:0]):
 *     0: EOCEV   - End Of Conversion Event (Software/START-triggered, Channel or Scan mode)
 *     1: EOCCEV  - End Of Conversion Cycle Event (Scan mode cycle complete)
 *     2: EOTCEV  - End Of Timer-Triggered Conversion Event (MFT16-1 pulse)
 *     3: OVFEV   - Data Overflow Event (CHNnDAT overwritten before read)
 *   Temperature / V2T events (bits[7:4], valid when CONFIG_ADC_V2T_NPCX):
 *     4: TEOCEV  - Temperature End Of Conversion Event (single temp channel)
 *     5: TEOCCEV - Temperature End Of Conversion Cycle Event (temp scan complete)
 *     6: TEOTCEV - Temperature End Of Timer-Triggered Conversion Event
 *     7: TOVFEV  - Temperature Data Overflow Event (TCHNnDAT overwritten before read)
 *   Bits[15:8]: Reserved
 */
#define NPCX_ADCSTS_EOCEV                     0
#define NPCX_ADCSTS_EOCCEV                    1
#define NPCX_ADCSTS_EOTCEV                    2
#define NPCX_ADCSTS_OVFEV                     3
#define NPCX_ADCSTS_TEOCEV                    4  /* V2T: Temperature End Of Conversion Event */
#define NPCX_ADCSTS_TEOCCEV                   5  /* V2T: Temperature End Of Conversion Cycle Event */
#define NPCX_ADCSTS_TEOTCEV                   6  /* V2T: Temperature End Of Timer-Triggered Conv Event */
#define NPCX_ADCSTS_TOVFEV                    7  /* V2T: Temperature Data Overflow Event */
/* ADCCNF bits */
#define NPCX_ADCCNF_ADCEN                     0  /* ADC module enable */
#define NPCX_ADCCNF_ADCMD_FIELD               FIELD(1, 2)  /* Conversion mode: 00=channel,01=scan */
#define NPCX_ADCCNF_ADCRPTC                   3  /* Repetitive conversion mode */
#define NPCX_ADCCNF_START                     4  /* Start conversion (cleared after start) */
#define NPCX_ADCCNF_ADCTTE                    5  /* Timer-triggered enable (MFT16-1 TA2) */
#define NPCX_ADCCNF_INTECEN                   6  /* Interrupt enable for EOCEV */
#define NPCX_ADCCNF_INTECCEN                  7  /* Interrupt enable for EOCCEV */
#define NPCX_ADCCNF_INTETCEN                  8  /* Interrupt enable for EOTCEV (spec: INTTECEN) */
#define NPCX_ADCCNF_INTOVFEN                  9  /* Interrupt enable for OVFEV */
#define NPCX_ADCCNF_STOP                      11 /* Stop conversion immediately (W1) */
/* CHNxDAT bits */
#define NPCX_CHNDAT_CHDAT_FIELD               FIELD(0, 10) /* 10-bit result: Vi=CHDAT*VFS/1024 */
#define NPCX_CHNDAT_NEW                       15 /* New data available (must clear for V2T) */
/* THRCTS bits - datasheet section 4.15.9 offset 0x1A:
 *   Bits[15:11]: Reserved
 *   Bit 10: THR3_IEN - Threshold 3 Interrupt Enable (R/W)
 *   Bit  9: THR2_IEN - Threshold 2 Interrupt Enable (R/W)
 *   Bit  8: THR1_IEN - Threshold 1 Interrupt Enable (R/W)
 *   Bit  7: ADC_EVENT - ADC Event Status: set=at least one ADC status bit is 1 (RO)
 *   Bits[6:3]: Reserved
 *   Bit  2: THR3_STS - Threshold 3 event status (R/W1C)
 *   Bit  1: THR2_STS - Threshold 2 event status (R/W1C)
 *   Bit  0: THR1_STS - Threshold 1 event status (R/W1C)
 */
#define NPCX_THRCTS_THR3_IEN                  10
#define NPCX_THRCTS_THR2_IEN                  9
#define NPCX_THRCTS_THR1_IEN                  8
#define NPCX_THRCTS_ADC_EVENT                 7  /* RO: set when any ADCSTS status bit is 1 */
#define NPCX_THRCTS_THR3_STS                  2
#define NPCX_THRCTS_THR2_STS                  1
#define NPCX_THRCTS_THR1_STS                  0
/* THR_DCTL bits */
#define NPCX_THR_DCTL_THRD_EN                 15 /* Dual threshold mode enable */
#define NPCX_THR_DCTL_THR_DVAL                FIELD(0, 10) /* Deassertion threshold value */

/*
 * THRCTLm register field definitions (datasheet section 4.15.9):
 * Each THRCTLm (m=1..3) is a 16-bit register laid out as:
 *   [15]    THEN   - Threshold detector enable
 *   [14]    L_H    - Direction: 0=event when above THRVAL, 1=event when at/below THRVAL
 *   [13:10] CHNSEL - Channel select (0-13, matches SADDR/ADCCS numbering)
 *   [9:0]   THRVAL - Assertion threshold value (10-bit ADC value)
 */
#define NPCX_THRCTL_THEN                      15             /* Threshold enable */
#define NPCX_THRCTL_L_H                       14             /* 0=above, 1=at-or-below */
#define NPCX_THRCTL_CHNSEL_FIELD              FIELD(10, 4)   /* Channel select [13:10] */
#define NPCX_THRCTL_THRVAL_FIELD              FIELD(0, 10)   /* Assertion threshold value [9:0] */

/*
 * ADC Channel-to-DEVALT pin select mapping (SCFG registers, set via pinctrl in Zephyr).
 *
 * DEVALT1 (SCFG offset 0x11h) - External voltage input pin select:
 *   [7] VIN7_SL  → Channel 4 (SADDR/CC4, V: VIN7)
 *   [6] Reserved  (VIN6 not implemented on NPCM5)
 *   [5] VIN5_SL  → Channel 3 (SADDR/CC3, V: VIN5)
 *   [4] Reserved  (ATX5VSB_SL, not connected to ADC on NPCM5)
 *   [3] VIN3_SL  → Channel 2 (SADDR/CC2, V: VIN3)
 *   [2] VIN2_SL  → Channel 1 (SADDR/CC1, V: VIN2)
 *   [1] VIN1_SL  → Channel 0 (SADDR/CC0, V: VIN1)
 *   [0] Reserved  (VIN0 not implemented on NPCM5)
 *
 * DEVALT2 (SCFG offset 0x12h) - External voltage input pin select:
 *   [7:4] Reserved
 *   [3] GPIOE0E1_SL → 0=I3C0_SB_CLK/SDA (default), 1=GPIOE0/E1 (not ADC)
 *   [2] VIN16_SL → Channel 7 (SADDR/CC7, V: VIN16)
 *   [1] VIN15_SL → Channel 6 (SADDR/CC6, V: VIN15)
 *   [0] VIN14_SL → Channel 5 (SADDR/CC5, V: VIN14)
 *
 * Channels 8-13 are internal power monitor inputs (VSBV2, VSBV1, VHIF, VBAT,
 * 3VSB, VCORE) and do NOT require DEVALT pin select configuration.
 */
#define NPCM_ADC_DEVALT1_OFFSET         0x11  /* SCFG Device Alternate Function 1 offset */
#define NPCM_ADC_DEVALT2_OFFSET         0x12  /* SCFG Device Alternate Function 2 offset */
/* DEVALT1: ADC external input pin select bits */
#define NPCM_ADC_DEVALT1_VIN1_SL        BIT(1)  /* CH0 → VIN1  (GPIO93, SADDR=0, CC0) */
#define NPCM_ADC_DEVALT1_VIN2_SL        BIT(2)  /* CH1 → VIN2  (GPIO94, SADDR=1, CC1) */
#define NPCM_ADC_DEVALT1_VIN3_SL        BIT(3)  /* CH2 → VIN3  (GPIO95, SADDR=2, CC2) */
#define NPCM_ADC_DEVALT1_VIN5_SL        BIT(5)  /* CH3 → VIN5  (GPIO87, SADDR=3, CC3) */
#define NPCM_ADC_DEVALT1_VIN7_SL        BIT(7)  /* CH4 → VIN7  (GPIO86, SADDR=4, CC4) */
/* DEVALT2: ADC external input pin select bits (bit[3] is non-ADC, do not set for ADC use) */
#define NPCM_ADC_DEVALT2_VIN14_SL       BIT(0)  /* CH5 → VIN14 (GPIO92, SADDR=5, CC5) */
#define NPCM_ADC_DEVALT2_VIN15_SL       BIT(1)  /* CH6 → VIN15 (GPIO91, SADDR=6, CC6) */
#define NPCM_ADC_DEVALT2_VIN16_SL       BIT(2)  /* CH7 → VIN16 (GPIO90, SADDR=7, CC7) */
#define NPCM_ADC_DEVALT2_GPIOE0E1_SL    BIT(3)  /* Non-ADC: 0=I3C0 pins, 1=GPIOE0/E1 (default=0) */

/* Speed register default values (datasheet section 4.15.7 Initializing the ADC, steps 3-5) */
#define NPCX_ADCCNF2_DEFAULT_VAL              0x8901  /* Speed 1 Register: Word at offset 0x20 */
#define NPCX_GENDLY_DEFAULT_VAL               0x0100  /* Speed 2 Register: Word at offset 0x22 */
#define NPCX_MEAST_DEFAULT_VAL                0x0405  /* Speed 3 Register: Word at offset 0x26 */

/* V2T (Voltage to Temperature) related definitions */
#ifdef CONFIG_ADC_V2T_NPCX
/* V2T Control register - Datasheet register map offset: 0x100 */
#define NPCX_V2T_CTRL_OFFSET                  0x100
#define V2T_CTRL(base) (*(volatile uint16_t *)((base) + NPCX_V2T_CTRL_OFFSET))

/* V2T_CTRL register bit fields (datasheet offset 0x100):
 *   Bits[15:7]: Reserved
 *   Bit 6: OVT_MODE  - 0=comparator mode (default), 1=interrupt mode
 *   Bit 5: TINTO_EN  - Temp interrupt from data overflow event (TOVFEV)
 *   Bit 4: TINTT_EN  - Temp interrupt from end of timer-triggered conversion (TEOTCEV)
 *   Bit 3: TINTCC_EN - Temp interrupt from end of cyclic-conversion event (TEOCCEV)
 *   Bit 2: TINTC_EN  - Temp interrupt from end of conversion event (TEOCEV)
 *   Bit 1: OVT_EN    - OVT function enable
 *   Bit 0: TEMP_EN   - Enable ADC voltage-to-temperature function
 */
#define NPCX_V2T_CTRL_TEMP_EN                 0  /* Enable ADC voltage-to-temperature function */
#define NPCX_V2T_CTRL_OVT_EN                  1  /* OVT function enable */
#define NPCX_V2T_CTRL_TINTC_EN                2  /* Temp interrupt from TEOCEV (spec: TINTC_EN) */
#define NPCX_V2T_CTRL_TINTCC_EN               3  /* Temp interrupt from TEOCCEV */
#define NPCX_V2T_CTRL_TINTT_EN                4  /* Temp interrupt from TEOTCEV */
#define NPCX_V2T_CTRL_TINTO_EN                5  /* Temp interrupt from TOVFEV (data overflow) */
#define NPCX_V2T_CTRL_OVT_MODE                6  /* 0=comparator mode (default), 1=interrupt mode */

/* ADC Temperature Channel Enable Register (TEMP_CH) - Datasheet offset: 0x102
 * Bits[15:8]: Reserved
 * Bits[7:0]:  TCH0_EN-TCH7_EN - enable temperature measurement for external VIN channels only:
 *   TCH0_EN=bit0 → Voltage CH0 (VIN1),  TCH1_EN=bit1 → Voltage CH1 (VIN2)
 *   TCH2_EN=bit2 → Voltage CH2 (VIN3),  TCH3_EN=bit3 → Voltage CH3 (VIN5)
 *   TCH4_EN=bit4 → Voltage CH4 (VIN7),  TCH5_EN=bit5 → Voltage CH5 (VIN14)
 *   TCH6_EN=bit6 → Voltage CH6 (VIN15), TCH7_EN=bit7 → Voltage CH7 (VIN16)
 * Note: Internal power channels (CC8-CC13) do NOT support V2T temperature measurement.
 */
#define NPCX_TEMP_CH_OFFSET                   0x102
#define TEMP_CH(base) (*(volatile uint16_t *)((base) + NPCX_TEMP_CH_OFFSET))

/* ADC OVT Status Register (OVT_STS) - Datasheet offset: 0x106 (W1C) */
#define NPCX_OVT_STS_OFFSET                   0x106
#define OVT_STS(base) (*(volatile uint16_t *)((base) + NPCX_OVT_STS_OFFSET))

/* ADC Temperature Channel n Data Buffer Registers (TCHNnDAT) - Datasheet offset 0x10Eh
 * Register map shows 14 entries: TCHN0DAT=0x10Eh..TCHN13DAT=0x128h (+2 each)
 * Only 8 temperature channels are usable (TCHN0-TCHN7, 0x10E-0x11C), because
 * TEMP_CH register (0x102) only has 8 enable bits (TCH0_EN-TCH7_EN).
 * TCHN8DAT-TCHN13DAT exist in the register map but cannot be enabled via TEMP_CH.
 * Register layout (RO):
 *   Bit  15:    NEW              - New temperature data available (cleared on read by hardware)
 *   Bits 14-11: Reserved
 *   Bits 10-3:  TCHDAT[10:3]    - Temperature Channel Data (integer part, 8-bit)
 *   Bits  2-0:  TCHDAT_Fraction - Temperature Channel Fraction Data (3-bit)
 * Full temperature = TCHDAT[10:3] . TCHDAT_Fraction (11-bit fixed point, 1 LSB = 0.125°C)
 * NOTE (per datasheet section 4.15.7): The voltage CHNDAT.NEW bit must be cleared
 * (by reading CHNDAT) before V2T can generate new temperature data for the next cycle.
 */
static inline uint32_t npcx_tchndat_offset(uint32_t ch)
{
	/* Register map: TCHN0DAT=0x10E ... TCHN13DAT=0x128 (14 entries, +2 each)
	 * Only ch=0..7 are usable (TEMP_CH has 8 enable bits). Formula is valid for 0..13.
	 */
	return (0x10E + (ch * 2));
}

#define NPCX_TCHNDAT_OFFSET(ch) npcx_tchndat_offset(ch)
#define TCHNDAT(base, ch) \
	(*(volatile uint16_t *)((base) + NPCX_TCHNDAT_OFFSET(ch)))
/* TCHNnDAT field macros */
#define NPCX_TCHNDAT_NEW                      15           /* New temperature data flag (RO) */
#define NPCX_TCHNDAT_TCHDAT_FIELD             FIELD(3, 8)  /* Integer temperature data [10:3] */
#define NPCX_TCHNDAT_FRACTION_FIELD           FIELD(0, 3)  /* Fraction data [2:0], 1 LSB=0.125°C */

/* Thermistor Offset table registers (THR_Offset0 to THR_Offset16)
 * Datasheet: THR_Offset0=0x12Eh, THR_Offset1=0x130h, ..., THR_Offset16=0x150h (17 entries)
 * Register layout (R/W):
 *   Bits[15:14]: Reserved
 *   Bits[13:0]:  THR_OFFSET - Thermistor table offset item
 */
static inline uint32_t npcx_thr_offset_reg(uint32_t n)
{
	return (0x12E + (n * 2));
}
#define THR_O(base, n)         (*(volatile uint16_t *)((base) + npcx_thr_offset_reg(n)))
#define NPCX_THR_OFFSET_FIELD  FIELD(0, 14)  /* Thermistor offset value [13:0] */

/* Thermistor Slope table registers (THR_Slope0 to THR_Slope16)
 * Datasheet: THR_Slope0=0x152h, THR_Slope1=0x154h, ..., THR_Slope16=0x172h (17 entries)
 * Register layout (R/W):
 *   Bits[15:10]: Reserved
 *   Bits[9:0]:   THR_SLOPE - Thermistor table slope item
 */
static inline uint32_t npcx_thr_slope_reg(uint32_t n)
{
	return (0x152 + (n * 2));
}
#define THR_S(base, n)         (*(volatile uint16_t *)((base) + npcx_thr_slope_reg(n)))
#define NPCX_THR_SLOPE_FIELD   FIELD(0, 10)  /* Thermistor slope value [9:0] */

/* Temperature Channel Threshold Control registers (TEMP_THR_CTL0 to TEMP_THR_CTL11)
 * Datasheet: TEMP_THR_CTL0=0x180h, TEMP_THR_CTL1=0x182h, ..., TEMP_THR_CTL11=0x196h (12 entries)
 * Register layout (R/W):
 *   Bit  15:    TEMP_THEN            - Threshold enable (1=enabled)
 *   Bits 14-11: Reserved
 *   Bits 10-3:  TEMP_THRVAL[10:3]   - Threshold value integer part
 *   Bits  2-0:  TEMP_THRVAL_Fraction - Threshold fraction (1 LSB = 0.125°C)
 */
static inline uint32_t npcx_temp_thrctl_offset(uint32_t ch)
{
	/* ch=0..11, TEMP_THR_CTL0=0x180, ..., TEMP_THR_CTL11=0x196 */
	return (0x180 + (ch * 2));
}
#define TEMP_THRCTL(base, ch)              (*(volatile uint16_t *)((base) + npcx_temp_thrctl_offset(ch)))
#define NPCX_TEMP_THRCTL_THEN             15            /* Temperature threshold enable */
#define NPCX_TEMP_THRCTL_THRVAL_FIELD     FIELD(3, 8)   /* Integer threshold value [10:3] */
#define NPCX_TEMP_THRCTL_FRACTION_FIELD   FIELD(0, 3)   /* Fraction threshold value [2:0] */

/* Temperature Channel Deassertion Threshold registers (TEMP_THR_DCTL0 to TEMP_THR_DCTL11)
 * Datasheet: TEMP_THR_DCTL0=0x1A0h, ..., TEMP_THR_DCTL11=0x1B6h (12 entries)
 * Register layout (R/W):
 *   Bit  15:    TEMP_THRD_EN           - Deassertion threshold enable (1=enabled)
 *   Bits 14-11: Reserved
 *   Bits 10-3:  TEMP_THR_DVAL[10:3]   - Deassertion threshold value integer part
 *   Bits  2-0:  TEMP_THR_DVAL_Fraction - Deassertion threshold fraction (1 LSB = 0.125°C)
 */
static inline uint32_t npcx_temp_thr_dctl_offset(uint32_t ch)
{
	/* ch=0..11, TEMP_THR_DCTL0=0x1A0, ..., TEMP_THR_DCTL11=0x1B6 */
	return (0x1A0 + (ch * 2));
}
#define TEMP_THR_DCTL(base, ch)              (*(volatile uint16_t *)((base) + npcx_temp_thr_dctl_offset(ch)))
#define NPCX_TEMP_THR_DCTL_THRD_EN          15            /* Temperature deassertion threshold enable */
#define NPCX_TEMP_THR_DCTL_DVAL_FIELD       FIELD(3, 8)   /* Integer deassertion value [10:3] */
#define NPCX_TEMP_THR_DCTL_FRACTION_FIELD   FIELD(0, 3)   /* Fraction deassertion value [2:0] */

/* ADC Temperature Post-Offset registers (ADCTD pair registers)
 * Datasheet: ADCTD1-0PO=0x1C0h, ADCTD3-2PO=0x1C2h, ADCTD5-4PO=0x1C4h, ADCTD7-6PO=0x1C6h,
 *            ADCTD9-8PO=0x1C8h, ADCTD11-10PO=0x1CAh
 * 6 pair-registers covering 12 temperature channels (TD0-TD11), R/W
 * Each register packs two 8-bit post-offset adjustments:
 *   Bits[15:8]: TDn+1_POST_OFFSET[7:0] - Temperature offset adjust for odd TD channel
 *   Bits[7:0]:  TDn_POST_OFFSET[7:0]   - Temperature offset adjust for even TD channel
 * Usage: pair=0 → ADCTD1-0PO (TD0,TD1), pair=1 → ADCTD3-2PO (TD2,TD3), ...,
 *        pair=5 → ADCTD11-10PO (TD10,TD11)
 */
static inline uint32_t npcx_adctd_po_offset(uint32_t pair)
{
	/* pair=0..5, ADCTD1-0PO=0x1C0, ..., ADCTD11-10PO=0x1CA */
	return (0x1C0 + (pair * 2));
}
#define ADCTD_PO(base, pair)             (*(volatile uint16_t *)((base) + npcx_adctd_po_offset(pair)))
#define NPCX_ADCTD_PO_LOW_FIELD          FIELD(0, 8)   /* TDn_POST_OFFSET (even channel) [7:0] */
#define NPCX_ADCTD_PO_HIGH_FIELD         FIELD(8, 8)   /* TDn+1_POST_OFFSET (odd channel) [15:8] */

#endif /* CONFIG_ADC_V2T_NPCX */

/* Threshold comparator V2 support */
#ifdef CONFIG_ADC_NPCX_CMP_V2
#define NPCX_THEN_OFFSET                      0x2C
#define THEN(base) (*(volatile uint16_t *)((base) + NPCX_THEN_OFFSET))
#endif

#endif /* CONFIG_ADC_NPCM_SARADC */

#endif /* _NUVOTON_NPCM5_REG_DEF_ADC_H */
