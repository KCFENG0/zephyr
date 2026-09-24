/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ADC_ADC_NPCM_SARADC_H_
#define ZEPHYR_INCLUDE_DRIVERS_ADC_ADC_NPCM_SARADC_H_

/**
 * @brief NPCM SARADC Driver APIs
 * @defgroup adc_npcm_saradc NPCM SARADC Driver APIs
 * @ingroup adc_interface
 * @{
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief NPCM ADC threshold parameter structure
 *
 * Structure used to configure threshold detection parameters.
 */
struct adc_npcm_threshold_param {
	/** ADC channel to monitor */
	uint8_t chnsel;
	/** Comparison direction: false = lower than threshold, true = higher than threshold */
	bool l_h;
	/** Threshold value (10-bit ADC value) */
	uint16_t thrval;
	/** Threshold value in millivolts */
	uint32_t val_mv;
	/** Work queue item to submit when threshold is triggered */
	struct k_work *work;
};

/**
 * @brief Set threshold control parameters
 *
 * Configures a threshold comparator with the specified parameters.
 *
 * @param dev Pointer to the ADC device
 * @param th_sel Threshold selector (0-2 for three thresholds)
 * @param param Pointer to threshold parameter structure
 *
 * @retval 0 on success
 * @retval -EINVAL if threshold selector is invalid or device not configured for thresholds
 * @retval -ENOTSUP if threshold feature is not supported
 */
int adc_npcm_threshold_ctrl_set_param(const struct device *dev, const uint8_t th_sel,
				      const struct adc_npcm_threshold_param *param);

/**
 * @brief Set dual threshold parameters
 *
 * Configures dual threshold mode with separate assertion and deassertion values.
 * This is useful for implementing hysteresis in threshold detection.
 *
 * @param dev Pointer to the ADC device
 * @param th_sel Threshold selector (0-2)
 * @param thr_dval Deassertion threshold value (10-bit ADC value)
 * @param l_h Comparison direction for deassertion
 *
 * @retval 0 on success
 * @retval -EINVAL if threshold selector is invalid
 * @retval -ENOTSUP if dual threshold mode is not supported
 */
int adc_npcm_threshold_ctrl_set_dual_param(const struct device *dev, const uint8_t th_sel,
					   uint16_t thr_dval, bool l_h);

/**
 * @brief Enable or disable threshold detection
 *
 * Enables or disables a specific threshold comparator. When enabled, the
 * threshold will trigger interrupts when the configured condition is met.
 *
 * @param dev Pointer to the ADC device
 * @param th_sel Threshold selector (0-2)
 * @param enable true to enable, false to disable
 *
 * @retval 0 on success
 * @retval -EINVAL if threshold selector is invalid
 * @retval negative error code on failure
 */
int adc_npcm_threshold_ctrl_enable(const struct device *dev, uint8_t th_sel, const bool enable);

/**
 * @brief Convert millivolts to threshold ADC value
 *
 * Converts a voltage in millivolts to the corresponding 10-bit ADC threshold
 * value based on the ADC's reference voltage.
 *
 * @param dev Pointer to the ADC device
 * @param val_mv Voltage value in millivolts
 * @param thrval Pointer to store the calculated threshold value
 *
 * @retval 0 on success
 * @retval -EINVAL if voltage exceeds reference voltage
 */
int adc_npcm_threshold_mv_to_thrval(const struct device *dev, uint32_t val_mv, uint32_t *thrval);

#ifdef CONFIG_ADC_V2T_NPCM

/**
 * @brief Set V2T (Voltage-to-Temperature) channels
 *
 * Configures which ADC channels should use the voltage-to-temperature
 * conversion feature. This is typically used for on-chip temperature sensors.
 *
 * @param dev Pointer to the ADC device
 * @param channel_mask Bit mask of channels to enable for V2T mode
 *
 * @retval 0 on success
 * @retval -EINVAL if channel mask is invalid
 * @retval -EBUSY if V2T is already enabled
 */
int adc_npcm_v2t_set_channels(const struct device *dev, uint32_t channel_mask);

/**
 * @brief Get configured V2T channels
 *
 * Returns the current V2T channel configuration.
 *
 * @param dev Pointer to the ADC device
 *
 * @return Bit mask of V2T-enabled channels
 */
uint32_t adc_npcm_v2t_get_channels(const struct device *dev);

/**
 * @brief Start V2T (Voltage-to-Temperature) conversion
 *
 * Starts the V2T conversion sequence for the channels previously configured
 * via adc_npcm_v2t_set_channels(). Sets up the ADC in repetitive scan mode
 * and enables the V2T temperature conversion interrupt.
 *
 * @param dev Pointer to the ADC device
 *
 * @retval 0 on success
 * @retval negative error code on failure
 */
int adc_npcm_v2t_start(const struct device *dev);

#endif /* CONFIG_ADC_V2T_NPCM */

/**
 * @brief Setup timer-triggered ADC conversion
 *
 * Configures the ADC to be triggered by a hardware timer for periodic
 * conversions on a specific channel.
 *
 * @param dev Pointer to the ADC device
 * @param channel ADC channel to convert
 *
 * @retval 0 on success
 * @retval -EINVAL if channel is invalid
 * @retval negative error code on failure
 */
int adc_npcm_timer_trigger_setup(const struct device *dev, uint8_t channel);

/**
 * @brief Stop timer-triggered ADC conversion
 *
 * Disables timer-triggered mode and returns the ADC to software-triggered mode.
 *
 * @param dev Pointer to the ADC device
 *
 * @retval 0 on success
 */
int adc_npcm_timer_trigger_stop(const struct device *dev);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_ADC_ADC_NPCM_SARADC_H_ */
