/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_uart

#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_npcm, CONFIG_UART_LOG_LEVEL);

/* NPCM register access helper */
#define NPCM_UART_IS_BIT_SET(reg, bit)         (((reg >> bit) & (0x1)) != 0)
#define NPCM_UART_GET_FIELD(reg, offset, size) (((reg) >> (offset)) & ((1 << (size)) - 1))

/*
 * Universal Asynchronous Receiver-Transmitter (UART) device registers
 */
struct uart_reg {
	/* 0x000: Transmit Data Buffer */
	volatile uint8_t UTBUF;
	volatile uint8_t reserved1;
	/* 0x002: Receive Data Buffer */
	volatile uint8_t URBUF;
	volatile uint8_t reserved2;
	/* 0x004: Interrupt Control */
	volatile uint8_t UICTRL;
	volatile uint8_t reserved3;
	/* 0x006: Status */
	volatile uint8_t USTAT;
	volatile uint8_t reserved4;
	/* 0x008: Frame Select */
	volatile uint8_t UFRS;
	volatile uint8_t reserved5;
	/* 0x00A: Mode Select */
	volatile uint8_t UMDSL;
	volatile uint8_t reserved6;
	/* 0x00C: Baud Rate Divisor */
	volatile uint8_t UBAUD;
	volatile uint8_t reserved7;
	/* 0x00E: Baud Rate Prescaler */
	volatile uint8_t UPSR;
	volatile uint8_t reserved8[7];
	/* 0x016: FIFO Control */
	volatile uint8_t UFCTRL;
	volatile uint8_t reserved9;
	/* 0x018: TX FIFO Current Level */
	volatile uint8_t UTXFLV;
	volatile uint8_t reserved10;
	/* 0x01A: RX FIFO Current Level */
	volatile uint8_t URXFLV;
	volatile uint8_t reserved11;
};

/* UART register fields */
#define NPCM_UICTRL_TBE      0
#define NPCM_UICTRL_RBF      1
#define NPCM_UICTRL_ETI      5
#define NPCM_UICTRL_ERI      6
#define NPCM_UICTRL_EEI      7
#define NPCM_USTAT_PE        0
#define NPCM_USTAT_FE        1
#define NPCM_USTAT_DOE       2
#define NPCM_USTAT_ERR       3
#define NPCM_USTAT_BKD       4
#define NPCM_USTAT_RB9       5
#define NPCM_USTAT_XMIP      6
#define NPCM_UFRS_CHAR_FIELD 0
#define NPCM_UFRS_STP        2
#define NPCM_UFRS_XB9        3
#define NPCM_UFRS_PSEL_FIELD 4
#define NPCM_UFRS_PEN        6
#define NPCM_UFCTRL_FIFOEN   0
#define NPCM_UTXFLV_TFL      0
#define NPCM_UTXFLV_TFL_SIZE 5
#define NPCM_URXFLV_RFL      0
#define NPCM_URXFLV_RFL_SIZE 5

/* UART baud register calculation */
#define NPCM_UART_BAUD_COFFICIENT         16
#define NPCM_UART_BAUD_PRESCALAR_CONST    10
#define NPCM_UART_BAUD_PRESCALAR_STEP     5
#define NPCM_UART_BAUD_PRESCALAR_STEP_MAX 31

/* UART baud registers */
#define NPCM_UART_UPSC_SHIFT         3
#define NPCM_UART_UPSC_MASK          0x1F
#define NPCM_UART_UPSC_OFFSET(n)     (((n) & NPCM_UART_UPSC_MASK) << NPCM_UART_UPSC_SHIFT)
#define NPCM_UART_UDIV10_8_MASK      0x7
#define NPCM_UART_UDIV10_8_OFFSET(n) ((n) & NPCM_UART_UDIV10_8_MASK)
#define NPCM_UART_UDIV7_0_MASK       0xFF
#define NPCM_UART_UDIV7_0_OFFSET(n)  ((n) & NPCM_UART_UDIV7_0_MASK)

/* Driver config */
struct uart_npcm_config {
	struct uart_reg *base;
	/* clock configuration */
	uint32_t clk_id;
	/* pinmux configuration */
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif
};

/* Driver data */
struct uart_npcm_data {
	/* Baud rate */
	uint32_t baud_rate;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif
#ifdef CONFIG_PM_DEVICE
	uint32_t pm_state;
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_npcm_tx_fifo_ready(const struct device *dev)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;

	/* True if the Tx FIFO contains some space available */
	return !(NPCM_UART_GET_FIELD(inst->UTXFLV, NPCM_UTXFLV_TFL, NPCM_UTXFLV_TFL_SIZE) >= 16);
}

static int uart_npcm_rx_fifo_available(const struct device *dev)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;

	/* True if at least one byte is in the Rx FIFO */
	return !(NPCM_UART_GET_FIELD(inst->URXFLV, NPCM_URXFLV_RFL, NPCM_URXFLV_RFL_SIZE) == 0);
}

static void uart_npcm_dis_all_tx_interrupts(const struct device *dev)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;

	/* Disable ETI (Enable Transmit Interrupt) interrupt */
	inst->UICTRL &= ~(BIT(NPCM_UICTRL_ETI));
}

static void uart_npcm_clear_rx_fifo(const struct device *dev)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;
	uint8_t scratch;

	/* Read all dummy bytes out from Rx FIFO */
	while (uart_npcm_rx_fifo_available(dev)) {
		scratch = inst->URBUF;
	}
}

static int uart_npcm_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;
	uint8_t tx_bytes = 0U;

	/* If Tx FIFO is still ready to send */
	while ((size - tx_bytes > 0) && uart_npcm_tx_fifo_ready(dev)) {
		/* Put a character into Tx FIFO */
		inst->UTBUF = tx_data[tx_bytes++];
	}

	return tx_bytes;
}

static int uart_npcm_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;
	unsigned int rx_bytes = 0U;

	/* If least one byte is in the Rx FIFO */
	while ((size - rx_bytes > 0) && uart_npcm_rx_fifo_available(dev)) {
		/* Receive one byte from Rx FIFO */
		rx_data[rx_bytes++] = inst->URBUF;
	}

	return rx_bytes;
}

static void uart_npcm_irq_tx_enable(const struct device *dev)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;

	inst->UICTRL |= BIT(NPCM_UICTRL_ETI);
}

static void uart_npcm_irq_tx_disable(const struct device *dev)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;

	inst->UICTRL &= ~(BIT(NPCM_UICTRL_ETI));
}

static int uart_npcm_irq_tx_ready(const struct device *dev)
{
	return uart_npcm_tx_fifo_ready(dev);
}

static int uart_npcm_irq_tx_complete(const struct device *dev)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;

	/* Tx FIFO is empty or last byte is sending */
	return !NPCM_UART_IS_BIT_SET(inst->USTAT, NPCM_USTAT_XMIP);
}

static void uart_npcm_irq_rx_enable(const struct device *dev)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;

	inst->UICTRL |= BIT(NPCM_UICTRL_ERI);
}

static void uart_npcm_irq_rx_disable(const struct device *dev)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;

	inst->UICTRL &= ~(BIT(NPCM_UICTRL_ERI));
}

static int uart_npcm_irq_rx_ready(const struct device *dev)
{
	return uart_npcm_rx_fifo_available(dev);
}

static void uart_npcm_irq_err_enable(const struct device *dev)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;

	inst->UICTRL |= BIT(NPCM_UICTRL_EEI);
}

static void uart_npcm_irq_err_disable(const struct device *dev)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;

	inst->UICTRL &= ~(BIT(NPCM_UICTRL_EEI));
}

static int uart_npcm_irq_is_pending(const struct device *dev)
{
	return (uart_npcm_irq_tx_ready(dev) || uart_npcm_irq_rx_ready(dev));
}

static int uart_npcm_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 1;
}

static void uart_npcm_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct uart_npcm_data *data = dev->data;

	data->user_cb = cb;
	data->user_data = cb_data;
}

static void uart_npcm_isr(const struct device *dev)
{
	struct uart_npcm_data *data = dev->data;

	if (data->user_cb) {
		data->user_cb(dev, data->user_data);
	}
}

/*
 * Poll-in implementation for interrupt driven config, forward call to
 * uart_npcm_fifo_read().
 */
static int uart_npcm_poll_in(const struct device *dev, unsigned char *c)
{
	return uart_npcm_fifo_read(dev, c, 1) ? 0 : -1;
}

/*
 * Poll-out implementation for interrupt driven config, forward call to
 * uart_npcm_fifo_fill().
 */
static void uart_npcm_poll_out(const struct device *dev, unsigned char c)
{
	while (!uart_npcm_fifo_fill(dev, &c, 1)) {
		continue;
	}
}

#else  /* !CONFIG_UART_INTERRUPT_DRIVEN */

/*
 * Poll-in implementation for byte mode config, read byte from URBUF if
 * available.
 */
static int uart_npcm_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;

	if (!NPCM_UART_IS_BIT_SET(inst->UICTRL, NPCM_UICTRL_RBF)) {
		return -1;
	}

	*c = inst->URBUF;
	return 0;
}

/*
 * Poll-out implementation for byte mode config, write byte to UTBUF if empty.
 */
static void uart_npcm_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;

	while (!NPCM_UART_IS_BIT_SET(inst->UICTRL, NPCM_UICTRL_TBE)) {
		continue;
	}
	inst->UTBUF = c;
}
#endif /* !CONFIG_UART_INTERRUPT_DRIVEN */

/* UART api functions */
static int uart_npcm_err_check(const struct device *dev)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_reg *const inst = config->base;
	uint32_t err = 0U;

	uint8_t stat = inst->USTAT;

	if (NPCM_UART_IS_BIT_SET(stat, NPCM_USTAT_DOE)) {
		err |= UART_ERROR_OVERRUN;
	}

	if (NPCM_UART_IS_BIT_SET(stat, NPCM_USTAT_PE)) {
		err |= UART_ERROR_PARITY;
	}

	if (NPCM_UART_IS_BIT_SET(stat, NPCM_USTAT_FE)) {
		err |= UART_ERROR_FRAMING;
	}

	return err;
}

/* UART driver registration */
static const struct uart_driver_api uart_npcm_driver_api = {
	.poll_in = uart_npcm_poll_in,
	.poll_out = uart_npcm_poll_out,
	.err_check = uart_npcm_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_npcm_fifo_fill,
	.fifo_read = uart_npcm_fifo_read,
	.irq_tx_enable = uart_npcm_irq_tx_enable,
	.irq_tx_disable = uart_npcm_irq_tx_disable,
	.irq_tx_ready = uart_npcm_irq_tx_ready,
	.irq_tx_complete = uart_npcm_irq_tx_complete,
	.irq_rx_enable = uart_npcm_irq_rx_enable,
	.irq_rx_disable = uart_npcm_irq_rx_disable,
	.irq_rx_ready = uart_npcm_irq_rx_ready,
	.irq_err_enable = uart_npcm_irq_err_enable,
	.irq_err_disable = uart_npcm_irq_err_disable,
	.irq_is_pending = uart_npcm_irq_is_pending,
	.irq_update = uart_npcm_irq_update,
	.irq_callback_set = uart_npcm_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

void uart_set_npcm_baud_rate(const struct device *dev, int baud_rate, uint32_t src_clk)
{
	uint32_t div;
	uint32_t opt_dev;
	uint32_t min_deviation;
	uint32_t clk;
	uint32_t calc_baudrate;
	uint32_t deviation;
	const struct uart_npcm_config *const config = dev->config;
	struct uart_npcm_data *const data = dev->data;
	struct uart_reg *const inst = config->base;
	uint8_t prescalar;
	uint8_t opt_prescalar;
	uint8_t i;

	/*
	 * baud_rate calculation equation:
	 * BR = APB2_CLK/(16 x DIV x P)
	 * P is the prescalar decided by UPSC field (5 bits) in UPSR register.
	 * The correspondences between the 5-bit prescaler select (UPSC) and
	 * prescaler divide factors are as follows.
	 *  UPSC		Prescaler Factor
	 *  00000b	NO CLOCK
	 *  00001b	1
	 *  00010b	1.5
	 *  00011b	2
	 *  00100b	2.5
	 *  00101b	3
	 *  00110b	3.5
	 *  00111b	4
	 *  .
	 *  .
	 *  .
	 *  11110b	15.5
	 *  11111b	16
	 * The prescaler increased by 0.5.
	 * For easier calculation, the prescalar times 10.
	 */
	/* Calculated UART baudrate, clock source from APB2 */
	opt_prescalar = 0;
	opt_dev = 0;
	prescalar = NPCM_UART_BAUD_PRESCALAR_CONST;
	min_deviation = 0xFFFFFFFF;
	clk = src_clk;

	for (i = 1; i <= NPCM_UART_BAUD_PRESCALAR_STEP_MAX; i++) {
		div = (clk * NPCM_UART_BAUD_PRESCALAR_CONST) /
		      (NPCM_UART_BAUD_COFFICIENT * data->baud_rate * prescalar);
		if (div == 0) {
			div = 1;
		}

		calc_baudrate = (clk * NPCM_UART_BAUD_PRESCALAR_CONST) /
				(NPCM_UART_BAUD_COFFICIENT * div * prescalar);
		deviation = (calc_baudrate > data->baud_rate) ? (calc_baudrate - data->baud_rate)
							      : (data->baud_rate - calc_baudrate);
		if (deviation < min_deviation) {
			min_deviation = deviation;
			opt_prescalar = i;
			opt_dev = div;
		}
		prescalar += NPCM_UART_BAUD_PRESCALAR_STEP;
	}
	opt_dev--;
	inst->UPSR = NPCM_UART_UPSC_OFFSET(opt_prescalar) | NPCM_UART_UDIV10_8_OFFSET(opt_dev >> 8);
	inst->UBAUD = NPCM_UART_UDIV7_0_OFFSET(opt_dev);
}

static int uart_npcm_init(const struct device *dev)
{
	const struct uart_npcm_config *const config = dev->config;
	struct uart_npcm_data *const data = dev->data;
	struct uart_reg *const inst = config->base;
	const struct device *const clk_dev = DEVICE_DT_GET(DT_NODELABEL(pcc));
	uint32_t uart_rate;
	int ret;

	/* Turn on device clock first and get source clock freq. */
	ret = clock_control_on(clk_dev, (clock_control_subsys_t *)config->clk_id);
	if (ret < 0) {
		LOG_ERR("Turn on UART clock fail %d", ret);
		return ret;
	}

	ret = clock_control_get_rate(clk_dev, (clock_control_subsys_t *)config->clk_id, &uart_rate);
	if (ret < 0) {
		LOG_ERR("Get UART clock rate error %d", ret);
		return ret;
	}

	uart_set_npcm_baud_rate(dev, data->baud_rate, uart_rate);

	/*
	 * 8-N-1, FIFO enabled.
	 * Must be done after setting the divisor for the new divisor to take effect.
	 */
	inst->UFRS = 0x00;
#if CONFIG_UART_INTERRUPT_DRIVEN
	inst->UFCTRL |= BIT(NPCM_UFCTRL_FIFOEN);

	/* Disable all UART tx FIFO interrupts */
	uart_npcm_dis_all_tx_interrupts(dev);

	/* Clear UART rx FIFO */
	uart_npcm_clear_rx_fifo(dev);

	/* Configure UART interrupts */
	config->irq_config_func(dev);
#endif

	/* Configure pin-mux for uart device */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("UART pinctrl setup failed (%d)", ret);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static inline bool uart_npcm_device_is_transmitting(const struct device *dev)
{
	if (IS_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN)) {
		/* The transmitted transaction is completed? */
		return !uart_npcm_irq_tx_complete(dev);
	}

	/* No need for polling mode */
	return 0;
}

static inline int uart_npcm_get_power_state(const struct device *dev, uint32_t *state)
{
	const struct uart_npcm_data *const data = dev->data;

	*state = data->pm_state;
	return 0;
}

static inline int uart_npcm_set_power_state(const struct device *dev, uint32_t next_state)
{
	struct uart_npcm_data *const data = dev->data;

	/* If next device power state is LOW or SUSPEND power state */
	if (next_state == PM_DEVICE_STATE_LOW_POWER || next_state == PM_DEVICE_STATE_SUSPEND) {
		/*
		 * If uart device is busy with transmitting, the driver will
		 * stay in while loop and wait for the transaction is completed.
		 */
		while (uart_npcm_device_is_transmitting(dev)) {
			continue;
		}
	}

	data->pm_state = next_state;
	return 0;
}

/* Implements the device power management control functionality */
static int uart_npcm_pm_control(const struct device *dev, uint32_t ctrl_command, uint32_t *state,
				pm_device_cb cb, void *arg)
{
	int ret = 0;

	switch (ctrl_command) {
	case PM_DEVICE_STATE_SET:
		ret = uart_npcm_set_power_state(dev, *state);
		break;
	case PM_DEVICE_STATE_GET:
		ret = uart_npcm_get_power_state(dev, state);
		break;
	default:
		ret = -EINVAL;
	}

	if (cb != NULL) {
		cb(dev, ret, state, arg);
	}
	return ret;
}
#endif /* CONFIG_PM_DEVICE */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define NPCM_UART_IRQ_CONFIG_FUNC_DECL(inst)                                                       \
	static void uart_npcm_irq_config_##inst(const struct device *dev)
#define NPCM_UART_IRQ_CONFIG_FUNC_INIT(inst) .irq_config_func = uart_npcm_irq_config_##inst,
#define NPCM_UART_IRQ_CONFIG_FUNC(inst)                                                            \
	static void uart_npcm_irq_config_##inst(const struct device *dev)                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), uart_npcm_isr,        \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}
#else
#define NPCM_UART_IRQ_CONFIG_FUNC_DECL(inst)
#define NPCM_UART_IRQ_CONFIG_FUNC_INIT(inst)
#define NPCM_UART_IRQ_CONFIG_FUNC(inst)
#endif

#define NPCM_UART_INIT(inst)                                                                       \
	NPCM_UART_IRQ_CONFIG_FUNC_DECL(inst);                                                      \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static const struct uart_npcm_config uart_npcm_cfg_##inst = {                              \
		.base = (struct uart_reg *)DT_INST_REG_ADDR(inst),                                 \
		.clk_id = DT_PHA(DT_DRV_INST(inst), clocks, clk_id),                               \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
		NPCM_UART_IRQ_CONFIG_FUNC_INIT(inst)};                                             \
                                                                                                   \
	static struct uart_npcm_data uart_npcm_data_##inst = {                                     \
		.baud_rate = DT_INST_PROP(inst, current_speed),                                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &uart_npcm_init, NULL, &uart_npcm_data_##inst,                 \
			      &uart_npcm_cfg_##inst, PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,    \
			      &uart_npcm_driver_api);                                              \
                                                                                                   \
	NPCM_UART_IRQ_CONFIG_FUNC(inst)

DT_INST_FOREACH_STATUS_OKAY(NPCM_UART_INIT)
