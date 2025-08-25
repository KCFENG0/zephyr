/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <stdio.h>
#include <zephyr/device.h>
#include "gdma.h"
#define TEST_SIZE 100

/*-------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------*/
void gdma_memset_u8(uint8_t *dat, uint8_t set_val, uint32_t setlen)
{
	uint8_t val = set_val;

	if (setlen == 0) {
		return;
	}

	int key = irq_lock();

	GDMA_SRCB0 = (uint32_t)&val;
	GDMA_DSTB0 = (uint32_t)dat;
	GDMA_TCNT0 = setlen;
	GDMA_CTL0 = 0x10081;

	while (GDMA_CTL0 & 0x1) {
	}
	GDMA_CTL0 = 0;

	irq_unlock(key);
}

/*-------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------*/
void gdma_memcpy_u8(uint8_t *dst, uint8_t *src, uint32_t cpylen)
{
	if (cpylen == 0) {
		return;
	}

	int key = irq_lock();

	GDMA_SRCB0 = (uint32_t)src;
	GDMA_DSTB0 = (uint32_t)dst;
	GDMA_TCNT0 = cpylen;
	GDMA_CTL0 = 0x10001;

	while (GDMA_CTL0 & 0x1) {
	}
	GDMA_CTL0 = 0;

	irq_unlock(key);
}

/*-------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------*/
void gdma_memcpy_u32(uint8_t *dst, uint8_t *src, uint32_t cpylen)
{
	if (cpylen == 0) {
		return;
	}

	int key = irq_lock();

	GDMA_SRCB0 = (uint32_t)src;
	GDMA_DSTB0 = (uint32_t)dst;
	GDMA_TCNT0 = cpylen / 4;
	GDMA_CTL0 = 0x12001;

	while (GDMA_CTL0 & 0x1) {
	}
	GDMA_CTL0 = 0;

	irq_unlock(key);
}

/*-------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------*/
void gdma_memcpy_burst_u32(uint8_t *dst, uint8_t *src, uint32_t cpylen)
{
	uint32_t rlen;

	if (cpylen == 0) {
		return;
	}

	int key = irq_lock();

	/* src and dst address must 16byte aligned */
	if (((uint32_t)src & 0x0F) || ((uint32_t)dst & 0xF)) {
		gdma_memcpy_u8(dst, src, cpylen);
		irq_unlock(key);
		return;
	}

	/* aligned 64Byte length */
	rlen = cpylen & 0xFFFFFFC0;
	if (rlen) {
		FIU0_BURST_CFG = 0x0B;

		GDMA_SRCB0 = (uint32_t)src;
		GDMA_DSTB0 = (uint32_t)dst;
		GDMA_TCNT0 = rlen / 16;
		GDMA_CTL0 = 0x12201;

		while (GDMA_CTL0 & 0x1) {
		}
		GDMA_CTL0 = 0;

		FIU0_BURST_CFG = 0x03;

		src += rlen;
		dst += rlen;
		cpylen -= rlen;
	}

	/* remain length */
	if (cpylen) {
		gdma_memcpy_u8(dst, src, cpylen);
	}

	irq_unlock(key);
}

/*-------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------*/
void gdma_memcpy_u32_dstfix(uint8_t *dst, uint8_t *src, uint32_t cpylen)
{
	if (cpylen == 0) {
		return;
	}

	int key = irq_lock();

	GDMA_SRCB0 = (uint32_t)src;
	GDMA_DSTB0 = (uint32_t)dst;
	GDMA_TCNT0 = cpylen / 4;
	GDMA_CTL0 = 0x12041;

	while (GDMA_CTL0 & 0x1) {
	}
	GDMA_CTL0 = 0;

	irq_unlock(key);
}


/*-------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------------*/
void gdma_memcpy_u32_srcfix(uint8_t *dst, uint8_t *src, uint32_t cpylen)
{
	if (cpylen == 0) {
		return;
	}

	int key = irq_lock();

	GDMA_SRCB0 = (uint32_t)src;
	GDMA_DSTB0 = (uint32_t)dst;
	GDMA_TCNT0 = cpylen / 4;
	GDMA_CTL0 = 0x12081;

	while (GDMA_CTL0 & 0x1) {
	}
	GDMA_CTL0 = 0;

	irq_unlock(key);
}

void gdma_memset_u32_test(uint32_t *dat, uint32_t set_val, uint32_t setlen,int channel)
{
	uint32_t val = set_val;

	if (setlen == 0) {
		return;
	}
	int key = irq_lock();
	if (channel == 0){
		GDMA_SRCB0 = (uint32_t)&val;
		GDMA_DSTB0 = (uint32_t)dat;
		GDMA_TCNT0 = setlen;
		GDMA_CTL0 = GDMA_CTL0 | (BIT(NPCM_GDMA_GDMAEN) | BIT(NPCM_GDMA_SAFIX) | BIT(NPCM_GDMA_SOFTREQ)| BIT(NPCM_GDMA_TWS_13) | BIT(NPCM_GDMA_BME));
		while (GDMA_CTL0 & 0x1) {
		}
		GDMA_CTL0 = 0;
	}
	else if (channel == 1){
		GDMA_SRCB1 = (uint32_t)&val;
		GDMA_DSTB1 = (uint32_t)dat;
		GDMA_TCNT1 = setlen;
		GDMA_CTL1 = GDMA_CTL1 | (BIT(NPCM_GDMA_GDMAEN) | BIT(NPCM_GDMA_SAFIX) | BIT(NPCM_GDMA_SOFTREQ)| BIT(NPCM_GDMA_TWS_13)  | BIT(NPCM_GDMA_BME));
		while (GDMA_CTL1 & 0x1) {
		}
		GDMA_CTL1 = 0;
	}

	irq_unlock(key);
}

void gdma_memset_u16_test(uint16_t *dat, uint16_t set_val, uint32_t setlen,int channel)
{
	uint16_t val = set_val;

	if (setlen == 0) {
		return;
	}
	int key = irq_lock();
	if (channel == 0){
		GDMA_SRCB0 = (uint32_t)&val;
		GDMA_DSTB0 = (uint32_t)dat;
		GDMA_TCNT0 = setlen;
		GDMA_CTL0 = GDMA_CTL0 | (BIT(NPCM_GDMA_GDMAEN) | BIT(NPCM_GDMA_SAFIX) | BIT(NPCM_GDMA_SOFTREQ)| BIT(NPCM_GDMA_TWS_12));
		while (GDMA_CTL0 & 0x1) {
		}
		GDMA_CTL0 = 0;
	}
	else if (channel == 1){
		GDMA_SRCB1 = (uint32_t)&val;
		GDMA_DSTB1 = (uint32_t)dat;
		GDMA_TCNT1 = setlen;
		GDMA_CTL1 = GDMA_CTL1 | (BIT(NPCM_GDMA_GDMAEN) | BIT(NPCM_GDMA_SAFIX) | BIT(NPCM_GDMA_SOFTREQ)| BIT(NPCM_GDMA_TWS_12));
		while (GDMA_CTL1 & 0x1) {
		}
		GDMA_CTL1 = 0;
	}

	irq_unlock(key);
}

void gdma_memset_u8_test(uint8_t *dat, uint8_t set_val, uint32_t setlen,int channel)
{
	uint8_t val = set_val;

	if (setlen == 0) {
		return;
	}

	int key = irq_lock();
	if (channel == 0){
		GDMA_SRCB0 = (uint32_t)&val;
		GDMA_DSTB0 = (uint32_t)dat;
		GDMA_TCNT0 = setlen;
		GDMA_CTL0 = GDMA_CTL0 | (BIT(NPCM_GDMA_GDMAEN) | BIT(NPCM_GDMA_SAFIX) | BIT(NPCM_GDMA_SOFTREQ));

		while (GDMA_CTL0 & 0x1) {
		}
		GDMA_CTL0 = 0;
	}
	else if(channel == 1){
		GDMA_SRCB1 = (uint32_t)&val;
		GDMA_DSTB1 = (uint32_t)dat;
		GDMA_TCNT1 = setlen;
		GDMA_CTL1 = GDMA_CTL1 | (BIT(NPCM_GDMA_GDMAEN) | BIT(NPCM_GDMA_SAFIX) | BIT(NPCM_GDMA_SOFTREQ));

		while (GDMA_CTL1 & 0x1) {
		}
		GDMA_CTL1 = 0;
	}

	irq_unlock(key);
}

void gdma_memcpy_u8_test(uint8_t *dst, uint8_t *src, uint32_t cpylen, int type, int channel)
{
	uint32_t val;
	if (cpylen == 0) {
		return;
	}

	int key = irq_lock();

	switch (type) {
		case 0:
			val= BIT(NPCM_GDMA_GDMAEN) | BIT(NPCM_GDMA_SOFTREQ);
			break;
		case 1:
			val= BIT(NPCM_GDMA_GDMAEN) | BIT(NPCM_GDMA_SOFTREQ) | BIT(NPCM_GDMA_SAFIX);
			break;
		case 2:
			val= BIT(NPCM_GDMA_GDMAEN) | BIT(NPCM_GDMA_SOFTREQ) | BIT(NPCM_GDMA_DADIR)| BIT(NPCM_GDMA_SADIR);
			break;
		case 3:
			val= BIT(NPCM_GDMA_GDMAEN) | BIT(NPCM_GDMA_SOFTREQ) |BIT(NPCM_GDMA_SIEN);
			break;
		default:
			val=0;
			break;
	}
	if (channel == 0) {
		GDMA_CTL0 = 0;
		GDMA_SRCB0 = (uint32_t)src;
		GDMA_DSTB0 = (uint32_t)dst;
		GDMA_TCNT0 = cpylen;
		GDMA_CTL0 = val;

		while (GDMA_CTL0 & 0x1) {
		}
		if(type == 3) {
			if((GDMA_CTL0 && BIT(NPCM_GDMA_TC)) != 1)
				printk("C0 TC test failed\n");
		}
		GDMA_CTL0 = 0;
	}
	else if (channel == 1) {
		GDMA_CTL1 = 0;
		GDMA_SRCB1 = (uint32_t)src;
		GDMA_DSTB1 = (uint32_t)dst;
		GDMA_TCNT1 = cpylen;
		GDMA_CTL1 = val;

		while (GDMA_CTL1 & 0x1) {
		}
		if(type == 3) {
			if((GDMA_CTL1 && BIT(NPCM_GDMA_TC)) != 1)
				printk("C1 TC test failed\n");
		}
		GDMA_CTL1 = 0;
	}
	irq_unlock(key);
}

int gdma_powerdown_test(int channel)
{
	uint32_t val;

	if (channel == 0){
		GDMA_CTL0=0;
		GDMA_CTL0 = GDMA_CTL0 | BIT(NPCM_GDMA_GPD);
		val = GDMA_SRCB0;
		val ++;
		GDMA_SRCB0 = val;
		if(GDMA_SRCB0 == val)
			return 1;
		GDMA_CTL0=0;
	}
	else if (channel == 1){
		GDMA_CTL1=0;
		GDMA_CTL1 = GDMA_CTL1 | BIT(NPCM_GDMA_GPD);
		val = GDMA_SRCB1;
		val ++;
		GDMA_SRCB1 = val;
		if(GDMA_SRCB1 == val)
			return 1;
		GDMA_CTL1=0;
	}

	return 0;
}

void gdma_test()
{
	uint8_t test[TEST_SIZE], test2[TEST_SIZE];
	uint16_t dsttest[20];
	uint32_t temp[16];
	int i,ret;

	for(i=0;i<TEST_SIZE;i++)
		test[i]=i;
	ret=0;
	gdma_memset_u8_test(test2,0,TEST_SIZE,0);

	gdma_memcpy_u8_test(test2,test,TEST_SIZE,0,0);
	for(i=0;i<TEST_SIZE;i++)
		if(test2[i]!=i){
			printk("gdma C0 8bit type 0 test failed\n");
			return;
		}

	if ((GDMA_SRCB0 + TEST_SIZE) != GDMA_CSRC0){
		printk("gdma CSRC0 test failed\n");
		return;
	}
	if ((GDMA_DSTB0 + TEST_SIZE) != GDMA_CDST0){
		printk("gdma CDST0 test failed\n");
		return;
	}
	if ((GDMA_TCNT0 - TEST_SIZE) != GDMA_CTCNT0){
		printk("gdma CTCNT0 test failed\n");
		return;
	}

	gdma_memcpy_u8_test(test2,&test[TEST_SIZE-1],TEST_SIZE,1,0);
	for(i=0;i<TEST_SIZE;i++)
		if(test2[i]!=(TEST_SIZE-1)){
			printk("gdma C0 8 bit type 1 test failed\n");
			return;
		}

	gdma_memcpy_u8_test(&test2[TEST_SIZE-1],&test[TEST_SIZE-1],TEST_SIZE,2,0);
	for(i=0;i<100;i++)
		if(test2[i]!=i){
			printk("gdma C0 8 bit type 2 test failed\n");
			return;
		}

	gdma_memset_u8_test(test2,0,TEST_SIZE,1);

	gdma_memcpy_u8_test(test2,test,TEST_SIZE,0,1);
	for(i=0;i<TEST_SIZE;i++)
		if(test2[i]!=i){
			printk("gdma C1 8 bit type 0 test failed\n");
			return;
		}

	if ((GDMA_SRCB1 + TEST_SIZE) != GDMA_CSRC1){
		printk("gdma CSRC1 test failed\n");
		return;
	}
	if ((GDMA_DSTB1 + TEST_SIZE) != GDMA_CDST1){
		printk("gdma CDST1 test failed\n");
		return;
	}
	if ((GDMA_TCNT1 - TEST_SIZE) != GDMA_CTCNT1){
		printk("gdma CTCNT1 test failed\n");
		return;
	}

	gdma_memcpy_u8_test(test2,&test[TEST_SIZE-1],TEST_SIZE,1,1);
	for(i=0;i<TEST_SIZE;i++)
		if(test2[i]!=(TEST_SIZE-1)){
			printk("gdma C1 8 bit type 1 test failed\n");
			return;
		}

	gdma_memcpy_u8_test(&test2[TEST_SIZE-1],&test[TEST_SIZE-1],TEST_SIZE,2,1);
	for(i=0;i<TEST_SIZE;i++){
		if(test2[i]!=i){
			printk("gdma C1 8 bit type 2 test failed\n");
			return;
		}
	}

	gdma_memcpy_u8_test(test2,test,TEST_SIZE,3,0);

	ret=gdma_powerdown_test(0);
	if (ret == 1){
		printk("gdma GPD C0 test failed\n");
		return;
	}

	ret=gdma_powerdown_test(1);
	if (ret == 1){
		printk("gdma GPD C1 test failed\n");
		return;
	}

	gdma_memcpy_u8_test(test2,test,TEST_SIZE,3,1);

	gdma_memset_u16_test(dsttest,0xffff,20,0);
	for(i=0;i<20;i++)
		if (dsttest[i] != 0xffff){
		printk("gdma 16 bit C0 test failed\n");
		return;
		}

	gdma_memset_u16_test(dsttest,0,20,1);
	for(i=0;i<20;i++)
		if (dsttest[i] != 0){
		printk("gdma 16 bit C1 test failed\n");
		return;
		}

	gdma_memset_u32_test(temp,0,16,0);
	for(i=0;i<16;i++)
		if (temp[i] != 0){
		printk("BME C0 test failed\n");
		return;
		}

	gdma_memset_u32_test(temp,0xffffffff,16,0);
	for(i=0;i<16;i++)
		if (temp[i] != 0xffffffff){
		printk("BME C1 test failed\n");
		return;
		}

	printk("gdma test pass\n");
}

#define GDMA_TEST_HELP					\
	("gdma test\n"					\
	 "Usage: gdma test")



SHELL_STATIC_SUBCMD_SET_CREATE(sub_gdma,

	SHELL_CMD_ARG(test, NULL, GDMA_TEST_HELP, gdma_test, 1, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(gdma, &sub_gdma, "GDMA shell commands", NULL);
