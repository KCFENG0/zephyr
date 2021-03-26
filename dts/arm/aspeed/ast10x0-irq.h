/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2021 ASPEED Technology Inc.
 */
#ifndef _AST10X0_IRQ_H_
#define _AST10X0_IRQ_H_

#define AST10X0_IRQ_DEFAULT_PRIORITY	1


#define INTR_MMC			0
#define INTR_RESV_1			1
#define INTR_MAC			2

#define INTR_UART5			8
#define INTR_USBDEV			9
#define INTR_RESV_10			10
#define INTR_GPIO			11
#define INTR_SCU			12

#define INTR_ESPI			42

#define INTR_UART1			47
#define INTR_UART2			48
#define INTR_UART3			49
#define INTR_UART4			50

#define INTR_UARTDMA			56
#define INTR_UART6			57
#define INTR_UART7			58
#define INTR_UART8			59
#define INTR_UART9			60
#define INTR_UART10			61
#define INTR_UART11			62
#define INTR_UART12			63
#define INTR_UART13			64

#endif /* #ifndef _AST10X0_IRQ_H_ */
