/*
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_espi_taf

#include <soc.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/espi_saf.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(espi_taf, CONFIG_ESPI_LOG_LEVEL);

struct espi_taf_npcm_config {
	uintptr_t base;
	uintptr_t mapped_addr;
	uintptr_t rx_plsz;
	enum NPCM_ESPI_TAF_ERASE_BLOCK_SIZE erase_sz;
	enum NPCM_ESPI_TAF_MAX_READ_REQ max_rd_sz;
};

struct espi_taf_npcm_data {
	sys_slist_t callbacks;
};

#define HAL_INSTANCE(dev)						\
	((struct espi_reg *)((const struct espi_taf_npcm_config *)	\
	(dev)->config)->base)

#define FLBASE_ADDR (							\
	GET_FIELD(inst->FLASHBASE, NPCM_FLASHBASE_FLBASE_ADDR)		\
	<< GET_FIELD_POS(NPCM_FLASHBASE_FLBASE_ADDR))

#define PRTR_BADDR(i) (							\
	GET_FIELD(inst->FLASH_PRTR_BADDR[i], NPCM_FLASH_PRTR_BADDR)	\
	<< GET_FIELD_POS(NPCM_FLASH_PRTR_BADDR))

#define PRTR_HADDR(i) (							\
	GET_FIELD(inst->FLASH_PRTR_HADDR[i], NPCM_FLASH_PRTR_HADDR)	\
	<< GET_FIELD_POS(NPCM_FLASH_PRTR_HADDR)) | 0xFFF;

/* Check access region of read request is protected or not */
static bool espi_taf_check_read_protect(const struct device *dev, uint32_t addr, uint32_t len,
					uint8_t tag)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	uint32_t flash_addr = addr;
	uint8_t i;
	uint16_t override_rd;
	uint32_t base, high;
	bool rdpr;

	flash_addr += FLBASE_ADDR;

	for (i = 0; i < CONFIG_ESPI_TAF_PR_NUM; i++) {
		base = PRTR_BADDR(i);
		high = PRTR_HADDR(i);

		rdpr = NPCM_IS_BIT_SET(inst->FLASH_PRTR_BADDR[i], NPCM_FRGN_RPR);
		override_rd = GET_FIELD(inst->FLASH_RGN_TAG_OVR[i], NPCM_FLASH_TAG_OVR_RPR);

		if (rdpr && !NPCM_IS_BIT_SET(override_rd, tag) &&
		    (base <= flash_addr + len - 1 && flash_addr <= high)) {
			return true;
		}
	}

	return false;
}

/* Check access region of write request is protected or not */
static bool espi_taf_check_write_protect(const struct device *dev, uint32_t addr,
					 uint32_t len, uint8_t tag)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	uint32_t flash_addr = addr;
	uint8_t i;
	uint16_t override_wr;
	uint32_t base, high;
	bool wrpr;

	flash_addr += FLBASE_ADDR;

	for (i = 0; i < CONFIG_ESPI_TAF_PR_NUM; i++) {
		base = PRTR_BADDR(i);
		high = PRTR_HADDR(i);

		wrpr = NPCM_IS_BIT_SET(inst->FLASH_PRTR_BADDR[i], NPCM_FRGN_WPR);
		override_wr = GET_FIELD(inst->FLASH_RGN_TAG_OVR[i], NPCM_FLASH_TAG_OVR_WPR);

		if (wrpr && !NPCM_IS_BIT_SET(override_wr, tag) &&
		    (base <= flash_addr + len - 1 && flash_addr <= high)) {
			return true;
		}
	}

	return false;
}

static int espi_taf_npcm_configure(const struct device *dev, const struct espi_saf_cfg *cfg)
{
	ARG_UNUSED(cfg);
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	struct espi_taf_npcm_config *config = ((struct espi_taf_npcm_config *)(dev)->config);

#if !defined(CONFIG_ESPI_NPCM_FLASH_SAFS_HW_MODE)
	inst->FLASHCTL &= ~BIT(NPCM_FLASHCTL_SAF_AUTO_READ);
#else
	inst->FLASHBASE = config->mapped_addr;
#endif
	return 0;
}

static int espi_taf_npcm_set_pr(const struct device *dev, const struct espi_saf_protection *pr)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	const struct espi_saf_pr *preg = pr->pregions;
	size_t n = pr->nregions;
	uint8_t regnum;
	uint32_t bitmask, offset;
	uint32_t rw_pr, override_rw;

	if ((dev == NULL) || (pr == NULL)) {
		return -EINVAL;
	}

	if (pr->nregions >= CONFIG_ESPI_TAF_PR_NUM) {
		return -EINVAL;
	}

	while (n--) {
		regnum = preg->pr_num;

		if (regnum >= CONFIG_ESPI_TAF_PR_NUM) {
			return -EINVAL;
		}

		rw_pr = preg->master_bm_we << NPCM_FRGN_WPR;
		rw_pr = rw_pr | (preg->master_bm_rd << NPCM_FRGN_RPR);

		if (preg->flags) {
			bitmask = BIT_MASK(GET_FIELD_SZ(NPCM_FLASH_PRTR_BADDR));
			offset = GET_FIELD_POS(NPCM_FLASH_PRTR_BADDR);
			inst->FLASH_PRTR_BADDR[regnum] = ((preg->start & bitmask) << offset)
							 | rw_pr;
			bitmask = BIT_MASK(GET_FIELD_SZ(NPCM_FLASH_PRTR_HADDR));
			offset = GET_FIELD_POS(NPCM_FLASH_PRTR_HADDR);
			inst->FLASH_PRTR_HADDR[regnum] = (preg->end & bitmask) << offset;
		}

		override_rw = (preg->override_r << 16) | preg->override_w;
		inst->FLASH_RGN_TAG_OVR[regnum] = override_rw;
		preg++;
	}

	return 0;
}

static int espi_taf_npcm_activate(const struct device *dev)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);

	inst->FLASHCTL |= BIT(NPCM_FLASHCTL_SAF_PROT_LOCK);

	return 0;
}

static bool espi_taf_npcm_channel_ready(const struct device *dev)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);

	if (!NPCM_IS_BIT_SET(inst->ESPICFG, NPCM_ESPICFG_FLCHANMODE)) {
		return false;
	}
	return true;
}

static int espi_taf_npcm_flash_read(const struct device *dev, struct espi_saf_packet *pckt)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	uint8_t tag = pckt->buf[0];

	if (NPCM_IS_BIT_SET(inst->FLASHCTL, NPCM_FLASHCTL_SAF_PROT_LOCK)) {
		if (espi_taf_check_read_protect(dev, pckt->flash_addr, pckt->len, tag)) {
			LOG_ERR("RD Access protect region");
			return -EINVAL;
		}
	}

	return 0;
}

static int espi_taf_npcm_flash_write(const struct device *dev, struct espi_saf_packet *pckt)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	uint8_t tag = pckt->buf[0];

	if (NPCM_IS_BIT_SET(inst->FLASHCTL, NPCM_FLASHCTL_SAF_PROT_LOCK)) {
		if (espi_taf_check_write_protect(dev, pckt->flash_addr, pckt->len, tag)) {
			LOG_ERR("WR Access protection region");
			return -EINVAL;
		}
	}

	return 0;
}

static int espi_taf_npcm_flash_erase(const struct device *dev, struct espi_saf_packet *pckt)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	uint8_t tag = pckt->buf[0];

	if (NPCM_IS_BIT_SET(inst->FLASHCTL, NPCM_FLASHCTL_SAF_PROT_LOCK)) {
		if (espi_taf_check_write_protect(dev, pckt->flash_addr, pckt->len, tag)) {
			LOG_ERR("ER Access protection region");
			return -EINVAL;
		}
	}

	return 0;
}

static int espi_taf_npcm_init(const struct device *dev)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	struct espi_taf_npcm_config *config = ((struct espi_taf_npcm_config *)(dev)->config);

	SET_FIELD(inst->FLASHCFG, NPCM_FLASHCFG_FLCAPA,
		  NPCM_FLASH_SHARING_CAP_SUPP_TAF_AND_CAF);
	SET_FIELD(inst->FLASHCFG, NPCM_FLASHCFG_TRGFLEBLKSIZE,
		  BIT(config->erase_sz));
	SET_FIELD(inst->FLASHCFG, NPCM_FLASHCFG_FLREQSUP,
		  config->max_rd_sz);
	inst->FLASHBASE = config->mapped_addr;

	SET_FIELD(inst->FLASHCFG, NPCM_FLASHCFG_TRGRPMCSUPP, 0x10);
	SET_FIELD(inst->FLASHCFG2, NPCM_FLASHCFG2_TRGRPMCDEV, 0x3);
	SET_FIELD(inst->FLASHCFG2, NPCM_FLASHCFG2_RPMC1COUNT, 0x4);
	SET_FIELD(inst->FLASHCFG2, NPCM_FLASHCFG2_RPMC1OP1CODE, 0x9B);
	SET_FIELD(inst->FLASHCFG3, NPCM_FLASHCFG3_RPMC2COUNT, 0x4);
	SET_FIELD(inst->FLASHCFG3, NPCM_FLASHCFG3_RPMC2OP1CODE, 0x9B);
	SET_FIELD(inst->FLASHCFG4, NPCM_FLASHCFG4_RPMC3COUNT, 0x4);
	SET_FIELD(inst->FLASHCFG4, NPCM_FLASHCFG4_RPMC3OP1CODE, 0x9B);
	SET_FIELD(inst->FLASHCFG4, NPCM_FLASHCFG4_RPMC4COUNT, 0x4);
	SET_FIELD(inst->FLASHCFG4, NPCM_FLASHCFG4_RPMC4OP1CODE, 0x9B);

	return 0;
}

static const struct espi_saf_driver_api espi_taf_npcm_driver_api = {
	.config = espi_taf_npcm_configure,
	.set_protection_regions = espi_taf_npcm_set_pr,
	.activate = espi_taf_npcm_activate,
	.get_channel_status = espi_taf_npcm_channel_ready,
	.flash_read = espi_taf_npcm_flash_read,
	.flash_write = espi_taf_npcm_flash_write,
	.flash_erase = espi_taf_npcm_flash_erase,
};

static struct espi_taf_npcm_data npcm_espi_taf_data;

static const struct espi_taf_npcm_config espi_taf_npcm_config = {
	.base = DT_INST_REG_ADDR(0),
	.mapped_addr = DT_INST_PROP(0, mapped_addr),
	.rx_plsz = DT_PROP(DT_INST_PARENT(0), rx_plsize),
	.erase_sz = DT_INST_STRING_TOKEN(0, erase_sz),
	.max_rd_sz = DT_INST_STRING_TOKEN(0, max_read_sz),
};

DEVICE_DT_INST_DEFINE(0, &espi_taf_npcm_init, NULL,
			&npcm_espi_taf_data, &espi_taf_npcm_config,
			PRE_KERNEL_2, CONFIG_ESPI_INIT_PRIORITY,
			&espi_taf_npcm_driver_api);
