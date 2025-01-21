/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_I3C_I3C_NPCM_H_
#define ZEPHYR_DRIVERS_I3C_I3C_NPCM_H_

#include <zephyr/toolchain.h>
#include <zephyr/types.h>
#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/gpio.h>

/* Operation type */
enum npcm_i3c_oper_state {
	I3C_OP_STATE_IDLE,
	I3C_OP_STATE_WR,
	I3C_OP_STATE_RD,
	I3C_OP_STATE_IBI,
	I3C_OP_STATE_MAX,
};

/* Control type */
enum npcm_i3c_mctrl_type {
	I3C_MCTRL_TYPE_I3C,
	I3C_MCTRL_TYPE_I2C,
	I3C_MCTRL_TYPE_I3C_HDR_DDR,
};

/* I3C timing configuration for each i3c/i2c speed */
struct npcm_i3c_timing_cfg {
	uint8_t ppbaud;   /* Push-Pull high period */
	uint8_t pplow;    /* Push-Pull low period */
	uint8_t odhpp;    /* Open-Drain high period */
	uint8_t odbaud;   /* Open-Drain low period */
	uint8_t i2c_baud; /* I2C period */
};

/* PDMA descriptor register */
struct pdma_dsct_reg {
	volatile uint32_t CTL;
	volatile uint32_t SA;
	volatile uint32_t DA;
	volatile uint32_t NEXT;
};

/* NPCM I3C Configuration */
struct npcm_i3c_config {
	/* Common I3C Driver Config */
	struct i3c_driver_config common;

	/* Pointer to controller registers. */
	struct i3c_reg *base;

	/* Clock control subsys related struct. */
	uint32_t clk_cfg;

	/* Pointer to pin control device. */
	const struct pinctrl_dev_config *pincfg;

	/* Interrupt configuration function. */
	void (*irq_config_func)(const struct device *dev);

	struct {
		uint32_t i3c_pp_scl_hz; /* I3C push pull clock frequency in Hz. */
		uint32_t i3c_od_scl_hz; /* I3C open drain clock frequency in Hz. */
		uint32_t i2c_scl_hz;    /* I2C clock frequency in Hz */
	} clocks;

#ifdef CONFIG_I3C_NPCM_DMA
	struct pdma_dsct_reg *pdma_rx;
	struct pdma_dsct_reg *pdma_tx;
#endif
};

/* NPCM I3C Data */
struct npcm_i3c_data {
	struct i3c_driver_data common; /* Common i3c driver data */
	struct k_mutex lock_mutex;     /* Mutex of i3c controller */
	struct k_sem sync_sem;         /* Semaphore used for synchronization */
	struct k_sem ibi_lock_sem;     /* Semaphore used for ibi */

	/* Target data */
	struct i3c_target_config *target_config;
	/* Configuration parameters for I3C hardware to act as target device */
	struct i3c_config_target config_target;
	struct k_sem target_lock_sem;       /* Semaphore used for i3c target */
	struct k_sem target_event_lock_sem; /* Semaphore used for i3c target ibi_raise() */

	enum npcm_i3c_oper_state oper_state; /* Operation state */

#ifdef CONFIG_I3C_NPCM_DMA
	uint8_t dma_rx_buf[4096];
	uint16_t dma_rx_len;
	uint16_t dma_triggered; /* The bit n is set to 1 if the n-th DMA channel is triggered */
	bool tx_valid;          /* Tx has valid data */
#else
	uint8_t rx_buf[4096];
	uint16_t rx_len;
	uint8_t *tx_buf;
	uint16_t tx_len;
#endif

#ifdef CONFIG_I3C_USE_IBI
	struct {
		/* List of addresses used in the MIBIRULES register. */
		uint8_t addr[5];

		/* Number of valid addresses in MIBIRULES. */
		uint8_t num_addr;

		/* True if all addresses have MSB set. */
		bool msb;

		/* True if all target devices require mandatory byte for IBI. */
		bool has_mandatory_byte;
	} ibi;
#endif

#ifdef CONFIG_I3C_NPCM_DMA
	struct pdma_dsct_reg dsct_sg[4] __aligned(4); /* use for dma, 4-bytes align */
#endif
};

/* NPCM I3C Register */
struct i3c_reg {
	/* 0x000: Controller Configuration */
	volatile uint32_t MCONFIG;
	/* 0x004: Target Configuration */
	volatile uint32_t CONFIG;
	/* 0x008: Target Status */
	volatile uint32_t STATUS;
	/* 0x00C: Target I3C Control */
	volatile uint32_t CTRL;
	/* 0x010: Target Interrupt Enable Set */
	volatile uint32_t INTSET;
	/* 0x014: Target Interrupt Enable Clear */
	volatile uint32_t INTCLR;
	/* 0x018: Target Interrupt Masked */
	volatile uint32_t INTMASKED;
	/* 0x01C: Target Error and Warning */
	volatile uint32_t ERRWARN;
	/* 0x020: Target DMA Control */
	volatile uint32_t DMACTRL;
	volatile uint32_t reserved1[2];
	/* 0x02C: Target Data Control */
	volatile uint32_t DATACTRL;
	/* 0x030: Target Write Byte Data */
	volatile uint32_t WDATAB;
	/* 0x034: Target Write Byte Data as End */
	volatile uint32_t WDATABE;
	/* 0x038: Target Write Half-Word Data */
	volatile uint32_t WDATAH;
	/* 0x03C: Target Write Half-Word Data as End */
	volatile uint32_t WDATAHE;
	/* 0x040: Target Read Byte Data */
	volatile uint32_t RDATAB;
	volatile uint32_t reserved2;
	/* 0x048: Target Read Half-Word Data */
	volatile uint32_t RDATAH;
	volatile uint32_t reserved3[2];
	/* 0x054: Target Byte-Only Write Byte Data */
	volatile uint8_t WDATAB1;
	volatile uint8_t reserved4[11];
	/* 0x060: Target Capabilities */
	volatile uint32_t CAPABILITIES;
	/* 0x064: Target Dynamic Address */
	volatile uint32_t DYNADDR;
	/* 0x068: Target Maximum Limits */
	volatile uint32_t MAXLIMITS;
	/* 0x06C: Target Part Number */
	volatile uint32_t PARTNO;
	/* 0x070: Target ID Extension */
	volatile uint32_t IDEXT;
	/* 0x074: Target Vendor ID */
	volatile uint32_t VENDORID;
	/* 0x078: Target Timing Control Clock */
	volatile uint32_t TCCLOCK;
	volatile uint32_t reserved5[2];
	/* 0x084: Controller Control */
	volatile uint32_t MCTRL;
	/* 0x088: Controller Status */
	volatile uint32_t MSTATUS;
	/* 0x08C: Controller IBI Registry and Rules */
	volatile uint32_t IBIRULES;
	/* 0x090: Controller Interrupt Enable Set */
	volatile uint32_t MINTSET;
	/* 0x094: Controller Interrupt Enable Clear */
	volatile uint32_t MINTCLR;
	/* 0x098: Controller Interrupt Masked */
	volatile uint32_t MINTMASKED;
	/* 0x09C: Controller Error and Warning */
	volatile uint32_t MERRWARN;
	/* 0x0A0: Controller DMA Control */
	volatile uint32_t MDMACTRL;
	volatile uint32_t reserved6[2];
	/* 0x0AC: Controller Data Control */
	volatile uint32_t MDATACTRL;
	/* 0x0B0: Controller Write Byte Data */
	volatile uint32_t MWDATAB;
	/* 0x0B4: Controller Write Byte Data as End */
	volatile uint32_t MWDATABE;
	/* 0x0B8: Controller Write Half-Word Data */
	volatile uint32_t MWDATAH;
	/* 0x0BC: Controller Write Half-Word Data as End */
	volatile uint32_t MWDATAHE;
	/* 0x0C0: Controller Read Byte Data */
	volatile uint32_t MRDATAB;
	volatile uint32_t reserved7;
	/* 0x0C8: Controller Read Half-Word Data */
	volatile uint32_t MRDATAH;
	/* 0x0CC: Controller Byte-Only Write Byte Data */
	volatile uint8_t MWDATAB1;
	volatile uint8_t reserved8[3];
	/* 0x0D0: Controller Start or Continue SDR Message */
	volatile uint32_t MWMSG_SDR;
	/* 0x0D4: Controller Read SDR Message Data */
	volatile uint32_t MRMSG_SDR;
	/* 0x0D8: Controller Start or Continue DDR Message */
	volatile uint32_t MWMSG_DDR;
	/* 0x0DC: Controller Read DDR Message Data */
	volatile uint32_t MRMSG_DDR;
	volatile uint32_t reserved9;
	/* 0x0E4: Controller Dynamic Address */
	volatile uint32_t MDYNADDR;
	volatile uint32_t reserved10[8];
	/* 0x108: Target HDR Command Register */
	volatile uint32_t HDRCMD;
	volatile uint32_t reserved11[13];
	/* 0x140: Target Extended IBI Data Register 1 */
	volatile uint32_t IBIEXT1;
	/* 0x144: Target Extended IBI Data Register 2 */
	volatile uint32_t IBIEXT2;
	volatile uint32_t reserved12[45];
	/* 0x1FC: Target Block ID */
	volatile uint32_t ID;
};

/* NPCM PDMA Register */
struct pdma_reg {
	/* 0x000 ~ 0x0DC: Descriptor Table Control Register 0 - 13 */
	struct pdma_dsct_reg PDMA_DSCT[14];
	volatile uint32_t reserved1[8];
	/* 0x100 ~ 0x134: Current Scatter-Gather Descriptor Table Address 0 - 13 */
	volatile uint32_t PDMA_CURSCAT[14];
	volatile uint32_t reserved2[178];
	/* 0x400: PDMA Channel Control Register */
	volatile uint32_t PDMA_CHCTL;
	/* 0x404: PDMA Stop Transfer Register */
	volatile uint32_t PDMA_STOP;
	/* 0x408: PDMA Software Request Register */
	volatile uint32_t PDMA_SWREQ;
	/* 0x40C: PDMA Request Active Flag Register */
	volatile uint32_t PDMA_TRGSTS;
	/* 0x410: PDMA Fixed Priority Setting Register */
	volatile uint32_t PDMA_PRISET;
	/* 0x414: PDMA Fixed Priority Clear Register */
	volatile uint32_t PDMA_PRICLR;
	/* 0x418: PDMA Interrupt Enable Control Register */
	volatile uint32_t PDMA_INTEN;
	/* 0x41C: PDMA PDMA Interrupt Status Register */
	volatile uint32_t PDMA_INTSTS;
	/* 0x420: PDMA Read/Write Target Abort Flag Register */
	volatile uint32_t PDMA_ABTSTS;
	/* 0x424: PDMA Transfer Done Flag Register */
	volatile uint32_t PDMA_TDSTS;
	/* 0x428: PDMA Scatter-Gather Transfer Done Flag Register */
	volatile uint32_t PDMA_SCATSTS;
	/* 0x42C: PDMA Transfer on Active Flag Register */
	volatile uint32_t PDMA_TACTSTS;
	volatile uint32_t reserved3[3];
	/* 0x43C: PDMA Scatter-Gather Descriptor Table Base Address Register */
	volatile uint32_t PDMA_SCATBA;
	volatile uint32_t reserved4[16];
	/* 0x480: PDMA Source Module Select Register 0 - 3 */
	volatile uint32_t PDMA_REQSEL[4];
};

/* NPCM PMC Register */
struct pmc_reg {
	/* 0x000: Power Management Controller */
	volatile uint8_t PMCSR;
	volatile uint8_t reserved1[2];
	/* 0x003: Enable in Sleep Control */
	volatile uint8_t ENIDL_CTL;
	/* 0x004: Disable in Idle Control */
	volatile uint8_t DISIDL_CTL;
	/* 0x005: Disable in Idle Control 1 */
	volatile uint8_t DISIDL_CTL1;
	volatile uint8_t reserved2;
	/* 0x007: Power-Down Control 0 */
	volatile uint8_t PWDWN_CTL0;
	/* 0x008: Power-Down Control 1 */
	volatile uint8_t PWDWN_CTL1;
	/* 0x009: Power-Down Control 2 */
	volatile uint8_t PWDWN_CTL2;
	/* 0x00A: Power-Down Control 3 */
	volatile uint8_t PWDWN_CTL3;
	/* 0x00B: Power-Down Control 4 */
	volatile uint8_t PWDWN_CTL4;
	/* 0x00C: Power-Down Control 5 */
	volatile uint8_t PWDWN_CTL5;
	/* 0x00D: Power-Down Control 6 */
	volatile uint8_t PWDWN_CTL6;
	volatile uint8_t reserved3[3];
	/* 0x011: RAM Power-Down Control 1 */
	volatile uint8_t RAM_PD1;
	/* 0x012: RAM Power-Down Control 2 */
	volatile uint8_t RAM_PD2;
	/* 0x013: Software Reset 1 */
	volatile uint8_t SW_RST1;
	/* 0x014: RAM Power-Down Control 3  */
	volatile uint8_t RAM_PD3;
	/* 0x015: Power-Down Control 7 */
	volatile uint8_t PWDWN_CTL7;
	/* 0x016: Power-Down Control 8 */
	volatile uint8_t PWDWN_CTL8;
};

/*
 * Register fields
 */
/* I3C Controller */
#define NPCM_I3C_MCONFIG_I2CBAUD        GENMASK(31, 28)
#define NPCM_I3C_MCONFIG_ODHPP          BIT(24)
#define NPCM_I3C_MCONFIG_ODBAUD         GENMASK(23, 16)
#define NPCM_I3C_MCONFIG_PPLOW          GENMASK(15, 12)
#define NPCM_I3C_MCONFIG_PPBAUD         GENMASK(11, 8)
#define NPCM_I3C_MCONFIG_ODSTOP         BIT(6)
#define NPCM_I3C_MCONFIG_DISTO          BIT(3)
#define NPCM_I3C_MCONFIG_MSTENA         GENMASK(1, 0)
#define NPCM_I3C_MCONFIG_MSTENA_OFF     0x0 /* Target Mode (set SLVENA) */
#define NPCM_I3C_MCONFIG_MSTENA_ON      0x1 /* Master Mode */
#define NPCM_I3C_MCONFIG_MSTENA_CAPABLE 0x2 /* Secondary Master Mode */

#define NPCM_I3C_MCTRL_RDTERM  GENMASK(23, 16)
#define NPCM_I3C_MCTRL_ADDR    GENMASK(15, 9)
#define NPCM_I3C_MCTRL_DIR     BIT(8)
#define NPCM_I3C_MCTRL_IBIRESP GENMASK(7, 6)
#define NPCM_I3C_MCTRL_IBIRESP_ACK                                                                 \
	0x0 /* ACK with mandatory byte determined by IBIRULES or ACK with no mandatory byte */
#define NPCM_I3C_MCTRL_IBIRESP_NACK          0x1 /* NACK */
#define NPCM_I3C_MCTRL_IBIRESP_ACK_MANDATORY 0x2 /* ACK with mandatory byte  */
#define NPCM_I3C_MCTRL_IBIRESP_MANUAL        0x3
#define NPCM_I3C_MCTRL_TYPE                  GENMASK(5, 4)
#define NPCM_I3C_MCTRL_REQUEST               GENMASK(2, 0)
#define NPCM_I3C_MCTRL_REQUEST_NONE          0x0 /* None */
#define NPCM_I3C_MCTRL_REQUEST_EMITSTARTADDR 0x1 /* Emit a START */
#define NPCM_I3C_MCTRL_REQUEST_EMITSTOP      0x2 /* Emit a STOP */
#define NPCM_I3C_MCTRL_REQUEST_IBIACKNACK    0x3 /* Manually ACK or NACK an IBI */
#define NPCM_I3C_MCTRL_REQUEST_PROCESSDAA    0x4 /* Starts the DAA process */
#define NPCM_I3C_MCTRL_REQUEST_FORCEEXIT     0x6 /* Emit HDR Exit Pattern  */
#define NPCM_I3C_MCTRL_REQUEST_AUTOIBI                                                             \
	0x7 /* Emits a START with address 7Eh when a slave pulls I3C_SDA low to request an IBI */

#define NPCM_I3C_MSTATUS_IBIADDR       GENMASK(30, 24)
#define NPCM_I3C_MSTATUS_NOWCNTLR      BIT(19)
#define NPCM_I3C_MSTATUS_ERRWARN       BIT(15)
#define NPCM_I3C_MSTATUS_IBIWON        BIT(13)
#define NPCM_I3C_MSTATUS_TXNOTFULL     BIT(12)
#define NPCM_I3C_MSTATUS_RXPEND        BIT(11)
#define NPCM_I3C_MSTATUS_COMPLETE      BIT(10)
#define NPCM_I3C_MSTATUS_MCTRLDONE     BIT(9)
#define NPCM_I3C_MSTATUS_SLVSTART      BIT(8)
#define NPCM_I3C_MSTATUS_IBITYPE       GENMASK(7, 6)
#define NPCM_I3C_MSTATUS_IBITYPE_NONE  0x0
#define NPCM_I3C_MSTATUS_IBITYPE_IBI   0x1
#define NPCM_I3C_MSTATUS_IBITYPE_CR    0x2
#define NPCM_I3C_MSTATUS_IBITYPE_HJ    0x3
#define NPCM_I3C_MSTATUS_NACKED        BIT(5)
#define NPCM_I3C_MSTATUS_BETWEEN       BIT(4)
#define NPCM_I3C_MSTATUS_STATE         GENMASK(2, 0)
#define NPCM_I3C_MSTATUS_STATE_IDLE    0x0
#define NPCM_I3C_MSTATUS_STATE_TGTREQ  0x1
#define NPCM_I3C_MSTATUS_STATE_NORMACT 0x3 /* SDR message mode */
#define NPCM_I3C_MSTATUS_STATE_MSGDDR  0x4
#define NPCM_I3C_MSTATUS_STATE_DAA     0x5
#define NPCM_I3C_MSTATUS_STATE_IBIACK  0x6
#define NPCM_I3C_MSTATUS_STATE_IBIRCV  0x7

#define NPCM_I3C_IBIRULES_NOBYTE     BIT(31)
#define NPCM_I3C_IBIRULES_MSB0       BIT(30)
#define NPCM_I3C_IBIRULES_ADDR4      GENMASK(29, 24)
#define NPCM_I3C_IBIRULES_ADDR3      GENMASK(23, 18)
#define NPCM_I3C_IBIRULES_ADDR2      GENMASK(17, 12)
#define NPCM_I3C_IBIRULES_ADDR1      GENMASK(11, 6)
#define NPCM_I3C_IBIRULES_ADDR0      GENMASK(5, 0)
#define NPCM_I3C_IBIRULES_ADDR_MSK   0x3F
#define NPCM_I3C_IBIRULES_ADDR_SHIFT 0x6

#define NPCM_I3C_MINTSET_NOWMASTER BIT(19)
#define NPCM_I3C_MINTSET_ERRWARN   BIT(15)
#define NPCM_I3C_MINTSET_IBIWON    BIT(13)
#define NPCM_I3C_MINTSET_TXNOTFULL BIT(12)
#define NPCM_I3C_MINTSET_RXPEND    BIT(11)
#define NPCM_I3C_MINTSET_COMPLETE  BIT(10)
#define NPCM_I3C_MINTSET_MCTRLDONE BIT(9)
#define NPCM_I3C_MINTSET_SLVSTART  BIT(8)

#define NPCM_I3C_MINTCLR_NOWMASTER BIT(19)
#define NPCM_I3C_MINTCLR_ERRWARN   BIT(15)
#define NPCM_I3C_MINTCLR_IBIWON    BIT(13)
#define NPCM_I3C_MINTCLR_TXNOTFULL BIT(12)
#define NPCM_I3C_MINTCLR_RXPEND    BIT(11)
#define NPCM_I3C_MINTCLR_COMPLETE  BIT(10)
#define NPCM_I3C_MINTCLR_MCTRLDONE BIT(9)
#define NPCM_I3C_MINTCLR_SLVSTART  BIT(8)

#define NPCM_I3C_MINTMASKED_NOWMASTER BIT(19)
#define NPCM_I3C_MINTMASKED_ERRWARN   BIT(15)
#define NPCM_I3C_MINTMASKED_IBIWON    BIT(13)
#define NPCM_I3C_MINTMASKED_TXNOTFULL BIT(12)
#define NPCM_I3C_MINTMASKED_RXPEND    BIT(11)
#define NPCM_I3C_MINTMASKED_COMPLETE  BIT(10)
#define NPCM_I3C_MINTMASKED_MCTRLDONE BIT(9)
#define NPCM_I3C_MINTMASKED_SLVSTART  BIT(8)

#define NPCM_I3C_MERRWARN_TIMEOUT BIT(20)
#define NPCM_I3C_MERRWARN_INVREQ  BIT(19)
#define NPCM_I3C_MERRWARN_MSGERR  BIT(18)
#define NPCM_I3C_MERRWARN_OWRITE  BIT(17)
#define NPCM_I3C_MERRWARN_OREAD   BIT(16)
#define NPCM_I3C_MERRWARN_HCRC    BIT(10)
#define NPCM_I3C_MERRWARN_HPAR    BIT(9)
#define NPCM_I3C_MERRWARN_TERM    BIT(4)
#define NPCM_I3C_MERRWARN_WRABT   BIT(3)
#define NPCM_I3C_MERRWARN_NACK    BIT(2)

#define NPCM_I3C_MDMACTRL_DMAWIDTH           GENMASK(5, 4)
#define NPCM_I3C_MDMACTRL_DMATB              GENMASK(3, 2)
#define NPCM_I3C_MDMACTRL_DMATB_DISABLE      0x0
#define NPCM_I3C_MDMACTRL_DMATB_EN_ONE_FRAME 0x1
#define NPCM_I3C_MDMACTRL_DMATB_EN_MANUAL    0x2
#define NPCM_I3C_MDMACTRL_DMAFB              GENMASK(1, 0)
#define NPCM_I3C_MDMACTRL_DMAFB_DISABLE      0x0
#define NPCM_I3C_MDMACTRL_DMAFB_EN_ONE_FRAME 0x1
#define NPCM_I3C_MDMACTRL_DMAFB_EN_MANUAL    0x2

#define NPCM_I3C_MDATACTRL_RXEMPTY BIT(31)
#define NPCM_I3C_MDATACTRL_TXFULL  BIT(30)
#define NPCM_I3C_MDATACTRL_RXCOUNT GENMASK(28, 24)
#define NPCM_I3C_MDATACTRL_TXCOUNT GENMASK(20, 16)
#define NPCM_I3C_MDATACTRL_RXTRIG  GENMASK(7, 6)
#define NPCM_I3C_MDATACTRL_TXTRIG  GENMASK(5, 4)
#define NPCM_I3C_MDATACTRL_UNLOCK  BIT(3)
#define NPCM_I3C_MDATACTRL_FLUSHFB BIT(1)
#define NPCM_I3C_MDATACTRL_FLUSHTB BIT(0)

#define NPCM_I3C_MWDATAB_END_A BIT(16)
#define NPCM_I3C_MWDATAB_END_B BIT(8)
#define NPCM_I3C_MWDATAB_DATA  GENMASK(7, 0)
#define NPCM_I3C_MWDATABE_DATA GENMASK(7, 0)

#define NPCM_I3C_MWDATAH_END   BIT(16)
#define NPCM_I3C_MWDATAH_DATA1 GENMASK(15, 8)
#define NPCM_I3C_MWDATAH_DATA0 GENMASK(7, 0)

#define NPCM_I3C_MWDATAHE_DATA1 GENMASK(15, 8)
#define NPCM_I3C_MWDATAHE_DATA0 GENMASK(7, 0)

#define NPCM_I3C_MRDATAB_DATA GENMASK(7, 0)

#define NPCM_I3C_MRDATAH_DATA1 GENMASK(15, 8)
#define NPCM_I3C_MRDATAH_DATA0 GENMASK(7, 0)

#define NPCM_I3C_MWDATAB1_DATA GENMASK(7, 0)

#define NPCM_I3C_MWMSG_SDR_CONTROL_LEN  GENMASK(15, 11)
#define NPCM_I3C_MWMSG_SDR_CONTROL_I2C  BIT(10)
#define NPCM_I3C_MWMSG_SDR_CONTROL_END  BIT(8)
#define NPCM_I3C_MWMSG_SDR_CONTROL_ADDR GENMASK(7, 1)
#define NPCM_I3C_MWMSG_SDR_CONTROL_DIR  BIT(0)

#define NPCM_I3C_MWMSG_SDR_DATA GENMASK(15, 0)

#define NPCM_I3C_MRMSG_SDR_DATA GENMASK(15, 0)

#define NPCM_I3C_MWMSG_DDR_CONTROL_END  BIT(14)
#define NPCM_I3C_MWMSG_DDR_CONTROL_LEN  GENMASK(9, 0)
#define NPCM_I3C_MWMSG_DDR_CONTROL_ADDR GENMASK(15, 9)
#define NPCM_I3C_MWMSG_DDR_CONTROL_DIR  BIT(7)
#define NPCM_I3C_MWMSG_DDR_CONTROL_CMD  GENMASK(6, 0)
#define NPCM_I3C_MWMSG_DDR_DATA         GENMASK(15, 0)
#define NPCM_I3C_MRMSG_DDR_DATA         GENMASK(15, 0)

#define NPCM_I3C_MDYNADDR_DADDR   GENMASK(1, 7)
#define NPCM_I3C_MDYNADDR_DAVALID BIT(0)

/* I3C Target */
#define NPCM_I3C_CONFIG_SADDR                 GENMASK(31, 25)
#define NPCM_I3C_CONFIG_BAMATCH               GENMASK(22, 16)
#define NPCM_I3C_CONFIG_HDRCMD                BIT(10)
#define NPCM_I3C_CONFIG_HDRCMD_RD_FROM_FIFO   0x0
#define NPCM_I3C_CONFIG_HDRCMD_RD_FROM_HDRCMD 0x1
#define NPCM_I3C_CONFIG_OFFLINE               BIT(9)
#define NPCM_I3C_CONFIG_IDRAND                BIT(8)
#define NPCM_I3C_CONFIG_DDROK                 BIT(4)
#define NPCM_I3C_CONFIG_S0IGNORE              BIT(3)
#define NPCM_I3C_CONFIG_MATCHSS               BIT(2)
#define NPCM_I3C_CONFIG_NACK                  BIT(1)
#define NPCM_I3C_CONFIG_SLVENA                BIT(0)

#define NPCM_I3C_STATUS_TIMECTRL              GENMASK(31, 30)
#define NPCM_I3C_STATUS_ACTSTATE              GENMASK(29, 28)
#define NPCM_I3C_STATUS_HJDIS                 BIT(27)
#define NPCM_I3C_STATUS_MRDIS                 BIT(25)
#define NPCM_I3C_STATUS_IBIDIS                BIT(24)
#define NPCM_I3C_STATUS_EVDET                 GENMASK(21, 20)
#define NPCM_I3C_STATUS_EVDET_NONE            0x0
#define NPCM_I3C_STATUS_EVDET_REQ_NOT_SENT    0x1
#define NPCM_I3C_STATUS_EVDET_REQ_SENT_NACKED 0x2
#define NPCM_I3C_STATUS_EVDET_REQ_SENT_ACKED  0x3
#define NPCM_I3C_STATUS_SLVSTART              BIT(19)
#define NPCM_I3C_STATUS_EVENT                 BIT(18)
#define NPCM_I3C_STATUS_CHANDLED              BIT(17)
#define NPCM_I3C_STATUS_DDRMATCH              BIT(16)
#define NPCM_I3C_STATUS_ERRWARN               BIT(15)
#define NPCM_I3C_STATUS_CCC                   BIT(14)
#define NPCM_I3C_STATUS_DACHG                 BIT(13)
#define NPCM_I3C_STATUS_TXNOTFULL             BIT(12)
#define NPCM_I3C_STATUS_RXPEND                BIT(11)
#define NPCM_I3C_STATUS_STOP                  BIT(10)
#define NPCM_I3C_STATUS_MATCHED               BIT(9)
#define NPCM_I3C_STATUS_START                 BIT(8)
#define NPCM_I3C_STATUS_STHDR                 BIT(6)
#define NPCM_I3C_STATUS_STDAA                 BIT(5)
#define NPCM_I3C_STATUS_STREQWR               BIT(4)
#define NPCM_I3C_STATUS_STREQRD               BIT(3)
#define NPCM_I3C_STATUS_STCCCH                BIT(2)
#define NPCM_I3C_STATUS_STMSG                 BIT(1)
#define NPCM_I3C_STATUS_STNOTSTOP             BIT(0)

#define NPCM_I3C_CTRL_VENDINFO        GENMASK(31, 24)
#define NPCM_I3C_CTRL_ACTSTATE        GENMASK(21, 20)
#define NPCM_I3C_CTRL_PENDINT         GENMASK(19, 16)
#define NPCM_I3C_CTRL_IBIDATA         GENMASK(15, 8)
#define NPCM_I3C_CTRL_EXTDATA         BIT(3)
#define NPCM_I3C_CTRL_EVENT           GENMASK(1, 0)
#define NPCM_I3C_CTRL_EVENT_NORMAL    0x0
#define NPCM_I3C_CTRL_EVENT_IBI       0x1
#define NPCM_I3C_CTRL_EVENT_CNTLR_REQ 0x2
#define NPCM_I3C_CTRL_EVENT_HJ        0x3

#define NPCM_I3C_INTSET_EVENT      BIT(18)
#define NPCM_I3C_INTSET_CHANDLED   BIT(17)
#define NPCM_I3C_INTSET_DDRMATCHED BIT(16)
#define NPCM_I3C_INTSET_ERRWARN    BIT(15)
#define NPCM_I3C_INTSET_CCC        BIT(14)
#define NPCM_I3C_INTSET_DACHG      BIT(13)
#define NPCM_I3C_INTSET_TXNOTFULL  BIT(12)
#define NPCM_I3C_INTSET_RXPEND     BIT(11)
#define NPCM_I3C_INTSET_STOP       BIT(10)
#define NPCM_I3C_INTSET_MATCHED    BIT(9)
#define NPCM_I3C_INTSET_START      BIT(8)

#define NPCM_I3C_INTCLR_EVENT      BIT(18)
#define NPCM_I3C_INTCLR_CHANDLED   BIT(17)
#define NPCM_I3C_INTCLR_DDRMATCHED BIT(16)
#define NPCM_I3C_INTCLR_ERRWARN    BIT(15)
#define NPCM_I3C_INTCLR_CCC        BIT(14)
#define NPCM_I3C_INTCLR_DACHG      BIT(13)
#define NPCM_I3C_INTCLR_TXNOTFULL  BIT(12)
#define NPCM_I3C_INTCLR_RXPEND     BIT(11)
#define NPCM_I3C_INTCLR_STOP       BIT(10)
#define NPCM_I3C_INTCLR_MATCHED    BIT(9)
#define NPCM_I3C_INTCLR_START      BIT(8)

#define NPCM_I3C_INTMASKED_SLVSTART   BIT(19)
#define NPCM_I3C_INTMASKED_EVENT      BIT(18)
#define NPCM_I3C_INTMASKED_CHANDLED   BIT(17)
#define NPCM_I3C_INTMASKED_DDRMATCHED BIT(16)
#define NPCM_I3C_INTMASKED_ERRWARN    BIT(15)
#define NPCM_I3C_INTMASKED_CCC        BIT(14)
#define NPCM_I3C_INTMASKED_DACHG      BIT(13)
#define NPCM_I3C_INTMASKED_TXNOTFULL  BIT(12)
#define NPCM_I3C_INTMASKED_RXPEND     BIT(11)
#define NPCM_I3C_INTMASKED_STOP       BIT(10)
#define NPCM_I3C_INTMASKED_MATCHED    BIT(9)
#define NPCM_I3C_INTMASKED_START      BIT(8)

#define NPCM_I3C_ERRWARN_OWRITE   BIT(17)
#define NPCM_I3C_ERRWARN_OREAD    BIT(16)
#define NPCM_I3C_ERRWARN_S0S1     BIT(11)
#define NPCM_I3C_ERRWARN_HCRC     BIT(10)
#define NPCM_I3C_ERRWARN_HPAR     BIT(9)
#define NPCM_I3C_ERRWARN_SPAR     BIT(8)
#define NPCM_I3C_ERRWARN_INVSTART BIT(4)
#define NPCM_I3C_ERRWARN_TERM     BIT(3)
#define NPCM_I3C_ERRWARN_URUNNACK BIT(2)
#define NPCM_I3C_ERRWARN_URUN     BIT(1)
#define NPCM_I3C_ERRWARN_ORUN     BIT(0)

#define NPCM_I3C_DMACTRL_DMAWIDTH           GENMASK(5, 4)
#define NPCM_I3C_DMACTRL_DMATB              GENMASK(3, 2)
#define NPCM_I3C_DMACTRL_DMATB_DISABLE      0x0
#define NPCM_I3C_DMACTRL_DMATB_EN_ONE_FRAME 0x1
#define NPCM_I3C_DMACTRL_DMATB_EN_MANUAL    0x2
#define NPCM_I3C_DMACTRL_DMAFB              GENMASK(1, 0)
#define NPCM_I3C_DMACTRL_DMAFB_DISABLE      0x0
#define NPCM_I3C_DMACTRL_DMAFB_EN_ONE_FRAME 0x1
#define NPCM_I3C_DMACTRL_DMAFB_EN_MANUAL    0x2

#define NPCM_I3C_DATACTRL_RXEMPTY BIT(31)
#define NPCM_I3C_DATACTRL_TXFULL  BIT(30)
#define NPCM_I3C_DATACTRL_RXCOUNT GENMASK(28, 24)
#define NPCM_I3C_DATACTRL_TXCOUNT GENMASK(20, 16)
#define NPCM_I3C_DATACTRL_RXTRIG  GENMASK(7, 6)
#define NPCM_I3C_DATACTRL_TXTRIG  GENMASK(5, 4)
#define NPCM_I3C_DATACTRL_UNLOCK  BIT(3)
#define NPCM_I3C_DATACTRL_FLUSHFB BIT(1)
#define NPCM_I3C_DATACTRL_FLUSHTB BIT(0)

#define NPCM_I3C_WDATAB_END_A BIT(16)
#define NPCM_I3C_WDATAB_END_B BIT(8)

#define NPCM_I3C_WDATABE_DATA GENMASK(7, 0)

#define NPCM_I3C_WDATAH_END   BIT(16)
#define NPCM_I3C_WDATAH_DATA1 GENMASK(15, 8)
#define NPCM_I3C_WDATAH_DATA0 GENMASK(7, 0)

#define NPCM_I3C_WDATAHE_DATA1 GENMASK(15, 8)
#define NPCM_I3C_WDATAHE_DATA0 GENMASK(7, 0)

#define NPCM_I3C_RDATAB_DATA GENMASK(7, 0)

#define NPCM_I3C_RDATAH_DATA1 GENMASK(15, 8)
#define NPCM_I3C_RDATAH_DATA0 GENMASK(7, 0)

#define NPCM_I3C_WDATAB1_DATA GENMASK(7, 0)

#define NPCM_I3C_CAPABILITIES_DMA       BIT(31)
#define NPCM_I3C_CAPABILITIES_INT       BIT(30)
#define NPCM_I3C_CAPABILITIES_FIFORX    GENMASK(29, 28)
#define NPCM_I3C_CAPABILITIES_FIFOTX    GENMASK(27, 26)
#define NPCM_I3C_CAPABILITIES_TIMECTRL  BIT(21)
#define NPCM_I3C_CAPABILITIES_IBI_MR_HJ GENMASK(20, 16)
#define NPCM_I3C_CAPABILITIES_CCCHANDLE GENMASK(15, 12)
#define NPCM_I3C_CAPABILITIES_SADDR     GENMASK(11, 10)
#define NPCM_I3C_CAPABILITIES_HDRSUPP   BIT(6)
#define NPCM_I3C_CAPABILITIES_IDREG     GENMASK(5, 2)
#define NPCM_I3C_CAPABILITIES_IDENA     GENMASK(1, 0)

#define NPCM_I3C_DYNADDR_DADDR   GENMASK(7, 1)
#define NPCM_I3C_DYNADDR_DAVALID BIT(0)

#define NPCM_I3C_MAXLIMITS_MAXWR GENMASK(27, 16)
#define NPCM_I3C_MAXLIMITS_MAXRD GENMASK(11, 0)

#define NPCM_I3C_PARTNO_PARTNO GENMASK(31, 0)

#define NPCM_I3C_IDEXT_BCR GENMASK(23, 16)
#define NPCM_I3C_IDEXT_DCR GENMASK(15, 8)

#define NPCM_I3C_VENDORID_VID GENMASK(14, 0)

#define NPCM_I3C_TCCLOCK_FREQ     GENMASK(15, 8)
#define NPCM_I3C_TCCLOCK_ACCURACY GENMASK(7, 0)

#define NPCM_I3C_IBIEXT1_EXT3 GENMASK(31, 24)
#define NPCM_I3C_IBIEXT1_EXT2 GENMASK(23, 16)
#define NPCM_I3C_IBIEXT1_EXT1 GENMASK(15, 8)
#define NPCM_I3C_IBIEXT1_MAX  GENMASK(6, 4)
#define NPCM_I3C_IBIEXT1_CNT  GENMASK(2, 0)

#define NPCM_I3C_IBIEXT2_EXT7 GENMASK(31, 24)
#define NPCM_I3C_IBIEXT2_EXT6 GENMASK(23, 16)
#define NPCM_I3C_IBIEXT2_EXT5 GENMASK(15, 8)
#define NPCM_I3C_IBIEXT2_EXT4 GENMASK(7, 0)

#define NPCM_I3C_HDRCMD_NEWCMD BIT(31)
#define NPCM_I3C_HDRCMD_OVFLW  BIT(30)
#define NPCM_I3C_HDRCMD_CMD0   GENMASK(7, 0)

#define NPCM_I3C_ID_ID GENMASK(31, 0)

/* PDMA */
#define NPCM_PDMA_INTSTS_TEIF  BIT(2)
#define NPCM_PDMA_INTSTS_TDIF  BIT(1)
#define NPCM_PDMA_INTSTS_ABTIF BIT(0)

#define NPCM_PDMA_SCATBA_16BITS GENMASK(31, 16)

#define NPCM_PDMA_REQSEL_CHANNEL(n) GENMASK((n * 8) + 7, n * 8)

#define NPCM_PDMA_DSCT_CTL_TXCNT         GENMASK(29, 16)
#define NPCM_PDMA_DSCT_CTL_TXWIDTH       GENMASK(13, 12)
#define NPCM_PDMA_DSCT_CTL_TX_WIDTH_8    0x0
#define NPCM_PDMA_DSCT_CTL_TX_WIDTH_16   0x1
#define NPCM_PDMA_DSCT_CTL_TX_WIDTH_32   0x2
#define NPCM_PDMA_DSCT_CTL_DAINC         GENMASK(11, 10)
#define NPCM_PDMA_DSCT_CTL_DAINC_FIX     0x3
#define NPCM_PDMA_DSCT_CTL_SAINC         GENMASK(9, 8)
#define NPCM_PDMA_DSCT_CTL_SAINC_FIX     0x3
#define NPCM_PDMA_DSCT_CTL_TBINTDIS      BIT(7)
#define NPCM_PDMA_DSCT_CTL_BURSIZE       GENMASK(6, 4)
#define NPCM_PDMA_DSCT_CTL_TXTYPE_SINGLE BIT(2)
#define NPCM_PDMA_DSCT_CTL_OPMODE        GENMASK(1, 0)
#define NPCM_PDMA_DSCT_CTL_OPMODE_STOP   0x0
#define NPCM_PDMA_DSCT_CTL_OPMODE_BASIC  0x1
#define NPCM_PDMA_DSCT_CTL_OPMODE_SGM    0x2

#define NPCM_PDMA_DSCT_NEXT_DSCT_OFFSET GENMASK(15, 2)

/* PMC */
#define NPCM_PMCSR_OLFC         BIT(7)
#define NPCM_PMCSR_OHFC         BIT(6)
#define NPCM_PMCSR_NWBSL        BIT(3)
#define NPCM_PMCSR_SLEEP        BIT(2)
#define NPCM_PMCSR_DHF          BIT(1)
#define NPCM_PMCSR_DS_INSTW     BIT(0)
#define NPCM_DISIDL_CTL_RAM_DID BIT(5)

#endif /* ZEPHYR_DRIVERS_I3C_I3C_NPCM_H_ */
