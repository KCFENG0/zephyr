/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_fiu_qspi

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/flash/npcm_flash_api_ex.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/dt-bindings/flash_controller/npcm_qspi.h>
#include <soc.h>

#include "flash_npcm_qspi.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(npcm_fiu_qspi, CONFIG_FLASH_LOG_LEVEL);

#define NPCM_FIU_CHK_TIMEOUT_US	10000

/*
 * NPCM500 physical CS to FIU register-role mapping:
 *
 * FIU0:
 *   SW_CS1 (SHD flash)  -> SHD  role: DEV_NUM=1 in UMA_CTS, SHD_4B (bit5), RD_CMD_SHD
 *   SW_CS2 (PVT flash)  -> BACK role: DEV_NUM_BACK=1 in UMA_ECTS, BACK_4B (bit6), RD_CMD_BACK
 * FIU1:
 *   SW_CS1 (HyperRAM)   -> SHD  role: DEV_NUM=1 in UMA_CTS, SHD_4B (bit5), RD_CMD_SHD
 * FIU2:
 *   SW_CS0 (MCP flash)  -> PVT  role: DEV_NUM=0 in UMA_CTS, PVT_4B (bit4), RD_CMD_PVT
 *
 * SW_CS0 was unused on NPCM400; on NPCM500 the private flash uses SW_CS2 in the
 * BACK register role. The DTS assigns qspi-flags accordingly.
 */
#define NPCM_FIU_SHD_CS		NPCM_QSPI_SW_CS1  /* SHD role: DEV_NUM=1 */
#define NPCM_FIU_BACK_CS	NPCM_QSPI_SW_CS2  /* BACK role: DEV_NUM_BACK=1 (PVT flash on NPCM500) */
#define NPCM_FIU_PVT_CS		NPCM_QSPI_SW_CS0  /* PVT role: DEV_NUM=0 (MCP flash on FIU2) */

/* Driver convenience defines */
#define HAL_INSTANCE(dev) \
	((struct fiu_reg *)((const struct npcm_qspi_fiu_config *)(dev)->config)->core_base)
#define HAL_HOST_INSTANCE(dev) \
	((struct fiu_reg *)((const struct npcm_qspi_fiu_config *)(dev)->config)->host_base)

/* Device config */
struct npcm_qspi_fiu_config {
	/* Flash controller core base address */
	uintptr_t core_base;
	/* Flash controller host base address */
	uintptr_t host_base;
	/* Clock configuration */
	uint32_t clk_cfg;
};

/* Clock control device */
struct npcm_scfg_config {
        /* scfg device base address */
        uintptr_t base_scfg;
};

static const struct npcm_scfg_config npcm_scfg_cfg = {
        .base_scfg = DT_REG_ADDR_BY_NAME(DT_NODELABEL(pinctrl), scfg),
};
#define NPCM_CLK_CTRL_DEV DT_NODELABEL(pcc)
#define NPCM_DEV_CTL3     0x04
#define DEV_CTL3_INST     (npcm_scfg_cfg.base_scfg + NPCM_DEV_CTL3)

#define NPCM_FIU_FLASH_WP       BIT(0)
#define NPCM_SPIM_FLASH_WP      BIT(1)
#define NPCM_DEV_CTL3_WP_INT_FL 0
/* NPCM500: DEV_CTL3[1] is renamed GPWRP vs NPCM400; bit position is unchanged */
#define NPCM_DEV_CTL3_WP_GPIO55 1 /* GPWRP on NPCM500 (GPIO55 write-protect) */
#define NPCM_DEV_CTL3_WP_GPIO76 2
static int npcm_pinctrl_flash_write_protect_set(int interface)
{
	volatile uintptr_t dev_ctl3 = DEV_CTL3_INST;
	uint32_t reg_val;

	switch (interface) {
	case NPCM_FIU_FLASH_WP:
		reg_val = sys_read32(dev_ctl3) | BIT(NPCM_DEV_CTL3_WP_GPIO55);
		sys_write32(reg_val, dev_ctl3);
		if (!(sys_read32(dev_ctl3) & BIT(NPCM_DEV_CTL3_WP_GPIO55))) {
			return -EIO;
		}
		break;

	case NPCM_SPIM_FLASH_WP:
		reg_val = sys_read32(dev_ctl3) | BIT(NPCM_DEV_CTL3_WP_INT_FL);
		sys_write32(reg_val, dev_ctl3);
		if (!(sys_read32(dev_ctl3) & BIT(NPCM_DEV_CTL3_WP_INT_FL))) {
			return -EIO;
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/* NPCM SPI User Mode Access (UMA) functions */
static inline void qspi_npcm_uma_cs_level(const struct device *dev, uint8_t sw_cs, bool level)
{
	struct fiu_reg *const inst = HAL_INSTANCE(dev);

	/* Set chip select to high/low level */
	if (level) {
		inst->UMA_ECTS |= BIT(sw_cs);
	} else {
		inst->UMA_ECTS &= ~BIT(sw_cs);
	}
}

static inline int qspi_npcm_uma_write_byte(const struct device *dev, uint8_t data)
{
	struct fiu_reg *const inst = HAL_INSTANCE(dev);
	struct npcm_qspi_data *const qspi_data = dev->data;
	const struct npcm_qspi_cfg *qspi_cfg = qspi_data->cur_cfg;
	int cts = 0;

	/* Set data to UMA_CODE and trigger UMA */
	inst->UMA_CODE = data;

	cts = UMA_CODE_ONLY_WRITE;

	/* share flash select, otherwise pvt or back */
	if (qspi_cfg->flags & NPCM_FIU_SHD_CS) {
		cts |= UMA_FLD_SHD_SL;
	} else {
		cts &= ~UMA_FLD_SHD_SL;
	}

	inst->UMA_CTS = cts;

	/* EXEC_DONE will be zero automatically if a UMA transaction is completed. */
	if (WAIT_FOR(!NPCM_IS_BIT_SET(inst->UMA_CTS, NPCM_UMA_CTS_EXEC_DONE),
		     NPCM_FIU_CHK_TIMEOUT_US, NULL) == false) {
		LOG_ERR("UMA write byte timeout");
		return -ETIMEDOUT;
	}

	return 0;
}

static inline int qspi_npcm_uma_read_byte(const struct device *dev, uint8_t *data)
{
	struct fiu_reg *const inst = HAL_INSTANCE(dev);
	struct npcm_qspi_data *const qspi_data = dev->data;
	const struct npcm_qspi_cfg *qspi_cfg = qspi_data->cur_cfg;
	int cts = 0;

	cts = UMA_CODE_ONLY_READ_BYTE(1);

	/* share flash select, otherwise pvt or back */
	if (qspi_cfg->flags & NPCM_FIU_SHD_CS) {
		cts |= UMA_FLD_SHD_SL;
	} else {
		cts &= ~UMA_FLD_SHD_SL;
	}

	/* Trigger UMA and Get data from DB0 later */
	inst->UMA_CTS = cts;

	if (WAIT_FOR(!NPCM_IS_BIT_SET(inst->UMA_CTS, NPCM_UMA_CTS_EXEC_DONE),
		     NPCM_FIU_CHK_TIMEOUT_US, NULL) == false) {
		LOG_ERR("UMA read byte timeout");
		return -ETIMEDOUT;
	}

	*data = inst->UMA_DB0;
	return 0;
}

/* NPCM SPI Direct Read Access (DRA)/User Mode Access (UMA) configuration functions */
static inline void qspi_npcm_config_uma_mode(const struct device *dev,
					     const struct npcm_qspi_cfg *qspi_cfg)
{
	struct fiu_reg *const inst = HAL_INSTANCE(dev);

	/*
	 * NPCM500 UMA chip-select selection:
	 *   SW_CS2 (BACK role, PVT flash on FIU0): set DEV_NUM_BACK in UMA_ECTS.
	 *     DEV_NUM in UMA_CTS is ignored when DEV_NUM_BACK=1.
	 *   SW_CS1 (SHD role): clear DEV_NUM_BACK, set DEV_NUM=1 in UMA_CTS
	 *     (handled in write/read_byte via UMA_FLD_SHD_SL).
	 *   SW_CS0 (PVT role, MCP flash on FIU2): clear DEV_NUM_BACK, DEV_NUM=0.
	 */
	if (qspi_cfg->flags & NPCM_FIU_BACK_CS) {
		inst->UMA_ECTS |= BIT(NPCM_UMA_ECTS_DEV_NUM_BACK);
	} else {
		inst->UMA_ECTS &= ~BIT(NPCM_UMA_ECTS_DEV_NUM_BACK);
	}
}

static inline void qspi_npcm_fiu_uma_release(const struct device *dev)
{
	struct fiu_reg *const inst = HAL_INSTANCE(dev);

	inst->FIU_MSR_IE_CFG &= ~BIT(NPCM_FIU_MSR_IE_CFG_UMA_BLOCK);
}

static inline int qspi_npcm_fiu_uma_lock(const struct device *dev)
{
	const struct npcm_qspi_fiu_config *cfg = dev->config;
	struct fiu_reg *const core_inst = HAL_INSTANCE(dev);

	/*
	 * Host arbitration only applies to FIU instances with a separate
	 * host register bank (FIU0: core 0x40020000 + host 0x40021000).
	 * FIU1/FIU2 are single-master -- no host interface -- so the
	 * MSTR_INACT bit is undefined and polling it would time out.
	 */
	if (cfg->host_base != cfg->core_base) {
		struct fiu_reg *const host_inst = HAL_HOST_INSTANCE(dev);

		if (WAIT_FOR(NPCM_IS_BIT_SET(host_inst->FIU_MSR_STS,
				NPCM_FIU_MSR_STS_MSTR_INACT),
				NPCM_FIU_CHK_TIMEOUT_US, NULL) == false) {
			LOG_ERR("wait host fiu inactive timeout");
			return -ETIMEDOUT;
		}
	}

	core_inst->FIU_MSR_IE_CFG |= BIT(NPCM_FIU_MSR_IE_CFG_UMA_BLOCK);

	return 0;
}

/* Enable/Disable 4-byte address mode for Direct Read Access (DRA) */
static inline void qspi_npcm_config_dra_4byte_mode(const struct device *dev,
						   const struct npcm_qspi_cfg *qspi_cfg)
{
#if defined(CONFIG_FLASH_NPCM_FIU_SUPP_DRA_4B_ADDR)
	struct fiu_reg *const core_inst = HAL_INSTANCE(dev);
	struct fiu_reg *const host_inst = HAL_HOST_INSTANCE(dev);
	uint8_t addr_4b_en = (qspi_cfg->flags & NPCM_QSPI_SW_CS_MASK) << 4;

	if (qspi_cfg->enter_4ba != 0) {
		core_inst->ADDR_4B_EN |= addr_4b_en;
		host_inst->ADDR_4B_EN |= addr_4b_en;
	} else {
		core_inst->ADDR_4B_EN &= ~addr_4b_en;
		host_inst->ADDR_4B_EN &= ~addr_4b_en;
	}
#endif /* CONFIG_FLASH_NPCM_FIU_SUPP_DRA_4B_ADDR */
}

int flash_npcm_fiu_set_4b_mode(const struct device *dev, bool enable)
{
#if defined(CONFIG_FLASH_NPCM_FIU_SUPP_DRA_4B_ADDR)
	struct fiu_reg *const core_inst = HAL_INSTANCE(dev);
	struct fiu_reg *const host_inst = HAL_HOST_INSTANCE(dev);
	struct npcm_qspi_data *qspi_data = dev->data;
	const struct npcm_qspi_cfg *qspi_cfg = qspi_data->cur_cfg;
	uint8_t addr_4b_en_mask;

	if (qspi_cfg == NULL) {
		return -EINVAL;
	}

	/*
	 * Bit mapping: SW_CS0->PVT_4B(bit4), SW_CS1->SHD_4B(bit5),
	 *              SW_CS2->BACK_4B(bit6). The << 4 shift is correct for
	 *              NPCM500 since SW_CS flags are BIT(0..2).
	 */
	addr_4b_en_mask = (qspi_cfg->flags & NPCM_QSPI_SW_CS_MASK) << 4;

	if (enable) {
		core_inst->ADDR_4B_EN |= addr_4b_en_mask;
		host_inst->ADDR_4B_EN |= addr_4b_en_mask;
	} else {
		core_inst->ADDR_4B_EN &= ~addr_4b_en_mask;
		host_inst->ADDR_4B_EN &= ~addr_4b_en_mask;
	}
#endif /* CONFIG_FLASH_NPCM_FIU_SUPP_DRA_4B_ADDR */

	return 0;
}

static inline void qspi_npcm_config_dra_mode(const struct device *dev,
					     const struct npcm_qspi_cfg *qspi_cfg)
{
	struct fiu_reg *const core_inst = HAL_INSTANCE(dev);
	struct fiu_reg *const host_inst = HAL_HOST_INSTANCE(dev);
	uint8_t rd_mode, rd_burst = NPCM_BURST_CFG_R_BURST_16B;

	switch (qspi_cfg->rd_mode) {
	case NPCM_RD_MODE_NORMAL:
		rd_mode = NPCM_SPI_FL_CFG_RD_MODE_NORMAL;
		break;
	case NPCM_RD_MODE_FAST:
		rd_mode = NPCM_SPI_FL_CFG_RD_MODE_FAST;
		break;
	case NPCM_RD_MODE_FAST_DUAL:
	case NPCM_RD_MODE_QUAD:
		rd_mode = NPCM_SPI_FL_CFG_RD_MODE_FAST_DUAL;
		break;
	default:
		LOG_ERR("unsupported rd mode: %d", qspi_cfg->rd_mode);
		return;
	}

	/* Select SPI read access type for Direct Read Access (DRA) mode;
	 * quad mode also requires RESP_CFG.QUAD_EN to be enabled below.
	 */
	SET_FIELD(core_inst->SPI_FL_CFG, NPCM_SPI_FL_CFG_RD_MODE, rd_mode);
	SET_FIELD(host_inst->SPI_FL_CFG, NPCM_SPI_FL_CFG_RD_MODE, rd_mode);

	if (qspi_cfg->rd_mode == NPCM_RD_MODE_QUAD) {
		/* Quad-I/O (4-1-4): RESP_CFG.QUAD_EN enables quad data path */
		core_inst->RESP_CFG |= BIT(NPCM_RESP_CFG_QUAD_EN);
		host_inst->RESP_CFG |= BIT(NPCM_RESP_CFG_QUAD_EN);
	} else {
		core_inst->RESP_CFG &= ~BIT(NPCM_RESP_CFG_QUAD_EN);
		host_inst->RESP_CFG &= ~BIT(NPCM_RESP_CFG_QUAD_EN);
	}

	/*
	 * NPCM500 QPI (4-4-4) mode for HyperRAM (FIU1):
	 * Per reference code fiu_dir_acc_quadrw(), Q_P_EN = 0xC1:
	 *   bit7: CMD_QUAD_EN   - command phase quad mode
	 *   bit6: ADDR_QUAD_EN  - address phase quad mode
	 *   bit0: QUAD_P_EN     - data phase quad for UMA write
	 *
	 * Per spec sec.4.1.7, UMA_QUADREAD (bit4) controls quad mode for the
	 * data phase of UMA read transactions.  The reference code omitted
	 * it because it only exercised DRA mode (where SPI_FL_CFG.RD_MODE
	 * + RESP_CFG.QUAD_EN govern the read data path instead).
	 * We set bit4 here so UMA reads also work in QPI 4-4-4 mode.
	 *
	 * Result: Q_P_EN = 0xD1 (bits 7,6,4,0)
	 */
	if (qspi_cfg->flags & NPCM_QSPI_QPI_MODE) {
		core_inst->Q_P_EN |= BIT(NPCM_Q_P_EN_CMD_QUAD_EN) |
				     BIT(NPCM_Q_P_EN_ADDR_QUAD_EN) |
				     BIT(NPCM_Q_P_EN_UMA_QUADREAD) |
				     BIT(NPCM_Q_P_EN_QUAD_P_EN);
		host_inst->Q_P_EN |= BIT(NPCM_Q_P_EN_CMD_QUAD_EN) |
				     BIT(NPCM_Q_P_EN_ADDR_QUAD_EN) |
				     BIT(NPCM_Q_P_EN_UMA_QUADREAD) |
				     BIT(NPCM_Q_P_EN_QUAD_P_EN);
		/*
		 * Enable SPI direct write (BURST_CFG.SPI_WR_EN, bit7).
		 * Per reference code: BURST_CFG = 0x83 (SPI_WR_EN | R_BURST=16B).
		 * Without this bit, AMBA writes to the HyperRAM address space
		 * (0x9000_0000) are silently dropped by the FIU.
		 * Per spec sec.4.1.7 hint #5: "Set up BURST_CFG" for direct_wr.
		 */
		core_inst->BURST_CFG |= BIT(NPCM_BURST_CFG_SPI_WR_EN);
		host_inst->BURST_CFG |= BIT(NPCM_BURST_CFG_SPI_WR_EN);
	} else {
		core_inst->Q_P_EN &= ~(BIT(NPCM_Q_P_EN_CMD_QUAD_EN) |
				       BIT(NPCM_Q_P_EN_ADDR_QUAD_EN) |
				       BIT(NPCM_Q_P_EN_UMA_QUADREAD) |
				       BIT(NPCM_Q_P_EN_QUAD_P_EN));
		host_inst->Q_P_EN &= ~(BIT(NPCM_Q_P_EN_CMD_QUAD_EN) |
				       BIT(NPCM_Q_P_EN_ADDR_QUAD_EN) |
				       BIT(NPCM_Q_P_EN_UMA_QUADREAD) |
				       BIT(NPCM_Q_P_EN_QUAD_P_EN));
	}

	/* Set read burst to max 16 bytes */
	SET_FIELD(core_inst->BURST_CFG, NPCM_BURST_CFG_R_BURST, rd_burst);
	SET_FIELD(host_inst->BURST_CFG, NPCM_BURST_CFG_R_BURST, rd_burst);

	/* Enable/Disable 4-byte address mode for Direct Read Access (DRA) */
	qspi_npcm_config_dra_4byte_mode(dev, qspi_cfg);
}

static inline void qspi_npcm_fiu_set_operation(const struct device *dev, uint32_t operation)
{
	if ((operation & NPCM_EX_OP_EXT_FLASH_WP) != 0) {
		npcm_pinctrl_flash_write_protect_set(NPCM_FIU_FLASH_WP);
	}
}

/* NPCM specific QSPI-FIU controller functions */
static int qspi_npcm_fiu_uma_transceive(const struct device *dev, struct npcm_transceive_cfg *cfg,
				     uint32_t flags)
{
	struct npcm_qspi_data *const data = dev->data;
    int ret = 0;

	/* Transaction is permitted? */
	if ((data->operation & NPCM_EX_OP_LOCK_TRANSCEIVE) != 0) {
        ret = -EPERM;
        goto exit;
	}

	/* UMA block */
	ret = qspi_npcm_fiu_uma_lock(dev);
	if (ret) {
        goto exit;
	}

	/* Assert chip select */
	qspi_npcm_uma_cs_level(dev, data->sw_cs, false);

	/* Transmit op-code first */
	ret = qspi_npcm_uma_write_byte(dev, cfg->opcode);
	if (ret != 0) {
		goto release_and_exit;
	}

	if ((flags & NPCM_TRANSCEIVE_ACCESS_ADDR) != 0) {
        size_t addr_bytes;
        const struct npcm_qspi_cfg *qspi_cfg = data->cur_cfg;

        if (qspi_cfg->enter_4ba != 0) {
            addr_bytes = 4;
        } else if (cfg->addr_count > 0) {
            addr_bytes = cfg->addr_count;
        } else {
            addr_bytes = 3;  /* Default to 3-byte addressing */
        }

        /* Calculate starting index for address bytes */
        const int addr_start = 4 - addr_bytes;

		for (size_t i = addr_start; i < 4; i++) {
			LOG_DBG("addr %d, %02x", i, cfg->addr.u8[i]);
			ret = qspi_npcm_uma_write_byte(dev, cfg->addr.u8[i]);
			if (ret != 0) {
				goto release_and_exit;
			}
		}
	}

	if ((flags & NPCM_TRANSCEIVE_ACCESS_WRITE) != 0) {
		if (cfg->tx_buf == NULL) {
            ret = -EINVAL;
            goto release_and_exit;
		}
		for (size_t i = 0; i < cfg->tx_count; i++) {
			ret = qspi_npcm_uma_write_byte(dev, cfg->tx_buf[i]);
			if (ret != 0) {
				goto release_and_exit;
			}
		}
	}

	if ((flags & NPCM_TRANSCEIVE_ACCESS_READ) != 0) {
		if (cfg->rx_buf == NULL) {
			ret = -EINVAL;
			goto release_and_exit;
		}
		for (size_t i = 0; i < cfg->rx_count; i++) {
			ret = qspi_npcm_uma_read_byte(dev, cfg->rx_buf + i);
			if (ret != 0) {
				goto release_and_exit;
			}
		}
	}

release_and_exit:
	/* De-assert chip select */
	qspi_npcm_uma_cs_level(dev, data->sw_cs, true);

	/* UMA unblock */
	qspi_npcm_fiu_uma_release(dev);

exit:
    return ret;
}

static void qspi_npcm_fiu_mutex_lock_configure(const struct device *dev,
					const struct npcm_qspi_cfg *cfg,
					const uint32_t operation)
{
	struct npcm_qspi_data *const data = dev->data;

	k_sem_take(&data->lock_sem, K_FOREVER);

	/* If the current device is different from previous one, configure it */
	if (data->cur_cfg != cfg) {
		data->cur_cfg = cfg;

		/* Apply pin-muxing and tri-state */
		pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);

		/* Save SW CS bit used in UMA mode */
		data->sw_cs = find_lsb_set(cfg->flags & NPCM_QSPI_SW_CS_MASK) - 1;

		/* Configure User Mode Access (UMA) settings */
		qspi_npcm_config_uma_mode(dev, cfg);

		/* Configure for Direct Read Access (DRA) settings */
		qspi_npcm_config_dra_mode(dev, cfg);
	}

	/* Set QSPI bus operation */
	if (data->operation != operation) {
		qspi_npcm_fiu_set_operation(dev, operation);
		data->operation = operation;
	}
}

static void qspi_npcm_fiu_mutex_unlock(const struct device *dev)
{
	struct npcm_qspi_data *const data = dev->data;

	k_sem_give(&data->lock_sem);
}

/* Enhanced RD_MODE configuration function */
static int qspi_npcm_fiu_set_read_mode(const struct device *dev, uint8_t rd_mode)
{
    struct npcm_qspi_data *const data = dev->data;
    struct fiu_reg *const core_inst = HAL_INSTANCE(dev);
    struct fiu_reg *const host_inst = HAL_HOST_INSTANCE(dev);
    uint8_t hw_rd_mode;
    int ret = 0;

    /* Validate read mode parameter */
    switch (rd_mode) {
        case NPCM_RD_MODE_NORMAL:
            hw_rd_mode = NPCM_SPI_FL_CFG_RD_MODE_NORMAL;
            break;
        case NPCM_RD_MODE_FAST:
            hw_rd_mode = NPCM_SPI_FL_CFG_RD_MODE_FAST;
            break;
        case NPCM_RD_MODE_FAST_DUAL:
            hw_rd_mode = NPCM_SPI_FL_CFG_RD_MODE_FAST_DUAL;
            break;
        case NPCM_RD_MODE_QUAD:
            hw_rd_mode = NPCM_SPI_FL_CFG_RD_MODE_FAST_DUAL; /* QUAD uses same as FAST_DUAL */
            break;
        default:
            ret = -EINVAL;
            goto exit;
    }

    /* Take semaphore to ensure atomic configuration */
    k_sem_take(&data->lock_sem, K_FOREVER);

    /* Update SPI_FL_CFG register RD_MODE field */
    SET_FIELD(core_inst->SPI_FL_CFG, NPCM_SPI_FL_CFG_RD_MODE, hw_rd_mode);
    SET_FIELD(host_inst->SPI_FL_CFG, NPCM_SPI_FL_CFG_RD_MODE, hw_rd_mode);

    /* Configure QUAD mode specific settings */
    if (rd_mode == NPCM_RD_MODE_QUAD) {
        core_inst->RESP_CFG |= BIT(NPCM_RESP_CFG_QUAD_EN);
        host_inst->RESP_CFG |= BIT(NPCM_RESP_CFG_QUAD_EN);
    } else {
        core_inst->RESP_CFG &= ~BIT(NPCM_RESP_CFG_QUAD_EN);
        host_inst->RESP_CFG &= ~BIT(NPCM_RESP_CFG_QUAD_EN);
    }

    /* Update current configuration if available */
    if (data->cur_cfg != NULL) {
        struct npcm_qspi_cfg *cfg = (struct npcm_qspi_cfg *)data->cur_cfg;

        cfg->rd_mode = rd_mode;
    }

    k_sem_give(&data->lock_sem);

exit:
    return ret;
}

/* Enhanced function to get current read mode */
static int qspi_npcm_fiu_get_read_mode(const struct device *dev, uint8_t *rd_mode)
{
    struct npcm_qspi_data *const data = dev->data;
    struct fiu_reg *const core_inst = HAL_INSTANCE(dev);
    uint8_t hw_rd_mode;
    int ret = 0;

    if (rd_mode == NULL) {
        ret = -EINVAL;
        goto exit;
    }

    /* Take semaphore for consistent read */
    k_sem_take(&data->lock_sem, K_FOREVER);

    /* Read current RD_MODE from SPI_FL_CFG register */
    hw_rd_mode = GET_FIELD(core_inst->SPI_FL_CFG, NPCM_SPI_FL_CFG_RD_MODE);

    /* Check if QUAD mode is enabled */
    bool quad_enabled = (core_inst->RESP_CFG & BIT(NPCM_RESP_CFG_QUAD_EN)) != 0;

    /* Convert hardware value to logical mode */
    if (quad_enabled && hw_rd_mode == NPCM_SPI_FL_CFG_RD_MODE_FAST_DUAL) {
        *rd_mode = NPCM_RD_MODE_QUAD;
    } else {
        switch (hw_rd_mode) {
            case NPCM_SPI_FL_CFG_RD_MODE_NORMAL:
                *rd_mode = NPCM_RD_MODE_NORMAL;
                break;
            case NPCM_SPI_FL_CFG_RD_MODE_FAST:
                *rd_mode = NPCM_RD_MODE_FAST;
                break;
            case NPCM_SPI_FL_CFG_RD_MODE_FAST_DUAL:
                *rd_mode = NPCM_RD_MODE_FAST_DUAL;
                break;
            default:
                *rd_mode = NPCM_RD_MODE_NORMAL; /* Default fallback */
                ret = -EIO;
                break;
        }
    }

    k_sem_give(&data->lock_sem);

exit:
    return ret;
}

/*
 * FIU CRC/Checksum Hardware Engine Operations
 *
 * The NPCM FIU provides hardware-accelerated CRC32 and Checksum calculation.
 * CRC engine control registers:
 * - CRCCON: Control register (mode, source, enable)
 * - CRCENT: Data entry register (write-only)
 * - CRCRSLT: Result register (read-only)
 *
 * CRCCON bits:
 *   [0] CALCEN  - Enable CRC calculation (1=enabled)
 *   [1] CKSMCRC - Mode selection (0=CRC32, 1=Checksum)
 *   [2] UMAENT  - Data source (0=Firmware, 1=UMA/Flash)
 */
static int qspi_npcm_fiu_crc_init(const struct device *dev, uint8_t mode,
				  uint8_t source, uint32_t initial_value)
{
	struct fiu_reg *const inst = HAL_INSTANCE(dev);
	struct npcm_qspi_data *const data = dev->data;

	k_sem_take(&data->lock_sem, K_FOREVER);

	/* Reset CRC control register */
	inst->CRCCON = 0;

	/* Initialize CRC result register with seed value (must be done before enabling) */
	inst->CRCRSLT = initial_value;

	/* Configure mode: bit 1 (CKSMCRC) - 0=CRC32, 1=Checksum */
	if (mode == NPCM_CRC_MODE_CHECKSUM) {
		inst->CRCCON |= BIT(1);
	}

	/* Configure source: bit 2 (UMAENT) - 0=Firmware, 1=UMA/Flash */
	if (source == NPCM_CRC_SRC_FLASH) {
		inst->CRCCON |= BIT(2);
	}

	/* Enable CRC calculation: bit 0 (CALCEN) */
	inst->CRCCON |= BIT(0);

	k_sem_give(&data->lock_sem);
	return 0;
}

static int qspi_npcm_fiu_crc_add_data(const struct device *dev,
				      const uint8_t *data_buf, size_t length)
{
	struct fiu_reg *const inst = HAL_INSTANCE(dev);
	struct npcm_qspi_data *const data = dev->data;

	if (data_buf == NULL || length == 0) {
		return -EINVAL;
	}

	k_sem_take(&data->lock_sem, K_FOREVER);

	/* Feed data bytes into CRC engine via CRCENT register */
	for (size_t i = 0; i < length; i++) {
		inst->CRCENT = data_buf[i];
	}

	k_sem_give(&data->lock_sem);
	return 0;
}

static int qspi_npcm_fiu_crc_get_result(const struct device *dev, uint32_t *result)
{
	struct fiu_reg *const inst = HAL_INSTANCE(dev);
	struct npcm_qspi_data *const data = dev->data;

	if (result == NULL) {
		return -EINVAL;
	}

	k_sem_take(&data->lock_sem, K_FOREVER);

	/* Read final CRC/Checksum result from CRCRSLT register */
	*result = inst->CRCRSLT;

	k_sem_give(&data->lock_sem);
	return 0;
}

struct npcm_qspi_ops npcm_qspi_fiu_ops = {
	.lock_configure = qspi_npcm_fiu_mutex_lock_configure,
        .unlock = qspi_npcm_fiu_mutex_unlock,
        .transceive = qspi_npcm_fiu_uma_transceive,
        .set_read_mode = qspi_npcm_fiu_set_read_mode,
        .get_read_mode = qspi_npcm_fiu_get_read_mode,
        .crc_init = qspi_npcm_fiu_crc_init,
        .crc_add_data = qspi_npcm_fiu_crc_add_data,
        .crc_get_result = qspi_npcm_fiu_crc_get_result,
};

static int qspi_npcm_fiu_init(const struct device *dev)
{
	const struct npcm_qspi_fiu_config *const config = dev->config;
	struct npcm_qspi_data *const data = dev->data;
	const struct device *const clk_dev = DEVICE_DT_GET(DT_NODELABEL(pcc));
	int ret;

	if (!device_is_ready(clk_dev)) {
		LOG_ERR("%s device not ready", clk_dev->name);
        ret = -ENODEV;
        goto exit;
	}

	/* Turn on device clock first and get source clock freq. */
	ret = clock_control_on(clk_dev,
			       (clock_control_subsys_t)config->clk_cfg);
	if (ret < 0) {
		LOG_ERR("Turn on FIU clock fail %d", ret);
        goto exit;
	}

	/* initialize mutex for qspi controller */
	k_sem_init(&data->lock_sem, 1, 1);

exit:
    return ret;
}

#define NPCM_SPI_FIU_INIT(n)							\
static const struct npcm_qspi_fiu_config npcm_qspi_fiu_config_##n = {		\
	.core_base = DT_INST_REG_ADDR_BY_IDX(n, 0),				\
	.host_base = DT_INST_REG_ADDR_BY_IDX(n, 1),				\
	.clk_cfg = DT_INST_PHA(n, clocks, clk_cfg),				\
};										\
static struct npcm_qspi_data npcm_qspi_data_##n = {				\
	.qspi_ops = &npcm_qspi_fiu_ops						\
};										\
DEVICE_DT_INST_DEFINE(n, qspi_npcm_fiu_init, NULL,				\
		      &npcm_qspi_data_##n, &npcm_qspi_fiu_config_##n,	\
		      PRE_KERNEL_1, CONFIG_FLASH_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(NPCM_SPI_FIU_INIT)
