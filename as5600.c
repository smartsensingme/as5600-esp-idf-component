#include "as5600.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "AS5600";

#if CONFIG_AS5600_THREAD_SAFE
/**
 * @brief Compute the FreeRTOS tick timeout for mutex acquisition.
 *
 * Converts the driver's millisecond timeout into FreeRTOS ticks.
 * If the configured timeout is negative, returns portMAX_DELAY (indefinite wait).
 * Otherwise ensures at least 1 tick is waited to prevent premature zero-tick drops.
 *
 * Called internally by driver functions before accessing I2C hardware.
 *
 * @param[in] dev Pointer to device context.
 * @return FreeRTOS tick count for xSemaphoreTake().
 */
static inline TickType_t as5600_get_lock_timeout(const as5600_dev_t *dev) {
  if (!dev || dev->transfer_timeout_ms < 0) {
    return portMAX_DELAY;
  }
  TickType_t ticks = pdMS_TO_TICKS(dev->transfer_timeout_ms);
  return ticks > 0 ? ticks : 1;
}
#endif

/**
 * @brief Read a single 8-bit register from the AS5600 over I2C.
 *
 * Sends the 1-byte register address with a repeated-start read of 1 byte data.
 *
 * Called internally by as5600_init_with_timeout() during setup.
 *
 * @param[in] dev Initialized device context.
 * @param[in] reg Register address to read.
 * @param[out] val Pointer receiving the 8-bit register value.
 * @return ESP_OK on success, or ESP-IDF I2C driver error code.
 */
static esp_err_t as5600_reg_read(as5600_dev_t *dev, uint8_t reg, uint8_t *val) {
  return i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, val, 1,
                                     dev->transfer_timeout_ms);
}

/**
 * @brief Write a single 8-bit register to the AS5600 over I2C.
 *
 * Packages the register address and value into a contiguous 2-byte transfer buffer.
 *
 * Called internally by as5600_init_with_timeout() during setup.
 *
 * @param[in] dev Initialized device context.
 * @param[in] reg Register address to write.
 * @param[in] val 8-bit value to write.
 * @return ESP_OK on success, or ESP-IDF I2C driver error code.
 */
static esp_err_t as5600_reg_write(as5600_dev_t *dev, uint8_t reg, uint8_t val) {
  uint8_t buffer[2] = {reg, val};
  return i2c_master_transmit(dev->i2c_dev, buffer, 2, dev->transfer_timeout_ms);
}

esp_err_t as5600_init(as5600_dev_t *dev, i2c_master_dev_handle_t i2c_dev,
                      const as5600_config_t *config) {
  /* Inherit timeout from config if positive, otherwise preserve indefinite waits. */
  int timeout = (config && config->transfer_timeout_ms > 0)
                    ? config->transfer_timeout_ms
                    : -1;
  return as5600_init_with_timeout(dev, i2c_dev, config, timeout);
}

esp_err_t as5600_init_with_timeout(as5600_dev_t *dev,
                                  i2c_master_dev_handle_t i2c_dev,
                                  const as5600_config_t *config,
                                  int timeout_ms) {
  /* Block 1: Argument validation */
  if (!dev || !i2c_dev || !config) {
    return ESP_ERR_INVALID_ARG;
  }
  if (timeout_ms < -1 || timeout_ms == 0 ||
      (timeout_ms > 0 && pdMS_TO_TICKS(timeout_ms) == 0)) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Block 2: Context initialization */
  dev->i2c_dev = i2c_dev;
  dev->config = *config;
  dev->transfer_timeout_ms = timeout_ms;

  /* Block 3: Concurrency management initialization */
#if CONFIG_AS5600_THREAD_SAFE
  dev->lock = xSemaphoreCreateMutexStatic(&dev->lock_buffer);
  if (dev->lock == NULL) {
    ESP_LOGE(TAG, "Failed to create static mutex");
    return ESP_ERR_NO_MEM;
  }
  if (xSemaphoreTake(dev->lock, as5600_get_lock_timeout(dev)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  /* Block 4: Verify sensor presence via STATUS register query */
  esp_err_t ret;
  uint8_t status;
  ret = as5600_reg_read(dev, AS5600_REG_STATUS, &status);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to communicate with AS5600 (err: %d)", ret);
    goto out;
  }

  /* Block 5: Read-modify-write configuration registers (CONF_H and CONF_L) */
  uint8_t conf_h, conf_l;
  uint8_t sf_bits;

  ret = as5600_reg_read(dev, AS5600_REG_CONF_H, &conf_h);
  if (ret != ESP_OK) goto out;

  ret = as5600_reg_read(dev, AS5600_REG_CONF_L, &conf_l);
  if (ret != ESP_OK) goto out;

  /*
   * Map slow filter factor (16, 8, 4, 2) to register bitfield (00, 01, 10, 11).
   * Note: The AS5600 hardware does not provide a 0x (disabled) setting; 2x is
   * the minimum internal slow-filter window supported by the silicon.
   */
  switch (config->slow_filter) {
    case 16: sf_bits = 0x00; break;
    case 8:  sf_bits = 0x01; break;
    case 4:  sf_bits = 0x02; break;
    case 2:  sf_bits = 0x03; break;
    default: sf_bits = 0x00; break;
  }

  /* Apply Fast Filter Threshold (bits 4:2) and Slow Filter (bits 1:0) */
  conf_h &= ~(AS5600_CONF_FTH_MASK | AS5600_CONF_SF_MASK);
  conf_h |= ((config->fast_filter_threshold << 3) & AS5600_CONF_FTH_MASK);
  conf_h |= ((sf_bits << 1) & AS5600_CONF_SF_MASK);

  /* Apply Hysteresis (bits 1:0 of CONF_L) */
  conf_l &= ~AS5600_CONF_HYST_MASK;
  conf_l |= (config->hysteresis & AS5600_CONF_HYST_MASK);

  ret = as5600_reg_write(dev, AS5600_REG_CONF_H, conf_h);
  if (ret != ESP_OK) goto out;

  ret = as5600_reg_write(dev, AS5600_REG_CONF_L, conf_l);
  if (ret != ESP_OK) goto out;

  ESP_LOGI(TAG, "Initialized successfully (Hysteresis: %d, Slow Filter: %dx, "
                "Fast Filter Threshold: %d, Timeout: %dms)",
           config->hysteresis, config->slow_filter,
           config->fast_filter_threshold, dev->transfer_timeout_ms);

out:
#if CONFIG_AS5600_THREAD_SAFE
  xSemaphoreGive(dev->lock);
#endif
  return ret;
}

esp_err_t as5600_set_timeout(as5600_dev_t *dev, int timeout_ms) {
  if (!dev) {
    return ESP_ERR_INVALID_ARG;
  }
  if (timeout_ms < -1 || timeout_ms == 0 ||
      (timeout_ms > 0 && pdMS_TO_TICKS(timeout_ms) == 0)) {
    return ESP_ERR_INVALID_ARG;
  }
  dev->transfer_timeout_ms = timeout_ms;
  return ESP_OK;
}

esp_err_t as5600_read_raw_angle(as5600_dev_t *dev, uint16_t *raw_angle) {
  if (!dev || !raw_angle) {
    return ESP_ERR_INVALID_ARG;
  }

#if CONFIG_AS5600_THREAD_SAFE
  if (xSemaphoreTake(dev->lock, as5600_get_lock_timeout(dev)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  /*
   * Read RAW_ANGLE_H (0x0C) and RAW_ANGLE_L (0x0D) in a single 2-byte burst.
   * Bits 11:8 are in buffer[0], bits 7:0 are in buffer[1]. Mask with 0x0FFF
   * ensures valid 12-bit range [0..4095].
   */
  uint8_t reg = AS5600_REG_RAW_ANGLE_H;
  uint8_t buffer[2];
  esp_err_t err = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, buffer, 2,
                                             dev->transfer_timeout_ms);
  if (err == ESP_OK) {
    *raw_angle = (((uint16_t)buffer[0] << 8) | buffer[1]) & 0x0FFF;
  }

#if CONFIG_AS5600_THREAD_SAFE
  xSemaphoreGive(dev->lock);
#endif
  return err;
}

esp_err_t as5600_read_angle(as5600_dev_t *dev, uint16_t *angle) {
  if (!dev || !angle) {
    return ESP_ERR_INVALID_ARG;
  }

#if CONFIG_AS5600_THREAD_SAFE
  if (xSemaphoreTake(dev->lock, as5600_get_lock_timeout(dev)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  /*
   * Read ANGLE_H (0x0E) and ANGLE_L (0x0F) in a single 2-byte burst.
   * Returns the filtered, scaled angle after internal DSP processing.
   */
  uint8_t reg = AS5600_REG_ANGLE_H;
  uint8_t buffer[2];
  esp_err_t err = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, buffer, 2,
                                             dev->transfer_timeout_ms);
  if (err == ESP_OK) {
    *angle = (((uint16_t)buffer[0] << 8) | buffer[1]) & 0x0FFF;
  }

#if CONFIG_AS5600_THREAD_SAFE
  xSemaphoreGive(dev->lock);
#endif
  return err;
}

esp_err_t as5600_read_status(as5600_dev_t *dev, uint8_t *status) {
  if (!dev || !status) {
    return ESP_ERR_INVALID_ARG;
  }

#if CONFIG_AS5600_THREAD_SAFE
  if (xSemaphoreTake(dev->lock, as5600_get_lock_timeout(dev)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  /* Query 1-byte magnet status register (0x0B). */
  uint8_t reg = AS5600_REG_STATUS;
  esp_err_t err = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, status, 1,
                                             dev->transfer_timeout_ms);

#if CONFIG_AS5600_THREAD_SAFE
  xSemaphoreGive(dev->lock);
#endif
  return err;
}

esp_err_t as5600_read_angles_and_status(as5600_dev_t *dev, uint16_t *raw_angle,
                                        uint16_t *angle, uint8_t *status) {
  if (!dev) {
    return ESP_ERR_INVALID_ARG;
  }

#if CONFIG_AS5600_THREAD_SAFE
  if (xSemaphoreTake(dev->lock, as5600_get_lock_timeout(dev)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }
#endif

  esp_err_t err = ESP_OK;

  /* Step 1: Read STATUS if requested */
  if (status) {
    uint8_t reg = AS5600_REG_STATUS;
    err = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, status, 1,
                                      dev->transfer_timeout_ms);
    if (err != ESP_OK) {
      goto out;
    }
  }

  /* Step 2: Read RAW_ANGLE (0x0C..0x0D) if requested */
  if (raw_angle) {
    uint8_t reg = AS5600_REG_RAW_ANGLE_H;
    uint8_t buffer[2];
    err = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, buffer, 2,
                                      dev->transfer_timeout_ms);
    if (err != ESP_OK) {
      goto out;
    }
    *raw_angle = (((uint16_t)buffer[0] << 8) | buffer[1]) & 0x0FFF;
  }

  /* Step 3: Read ANGLE (0x0E..0x0F) if requested */
  if (angle) {
    uint8_t reg = AS5600_REG_ANGLE_H;
    uint8_t buffer[2];
    err = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, buffer, 2,
                                      dev->transfer_timeout_ms);
    if (err != ESP_OK) {
      goto out;
    }
    *angle = (((uint16_t)buffer[0] << 8) | buffer[1]) & 0x0FFF;
  }

out:
#if CONFIG_AS5600_THREAD_SAFE
  xSemaphoreGive(dev->lock);
#endif
  return err;
}
