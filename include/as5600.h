#ifndef AS5600_H_
#define AS5600_H_

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#if CONFIG_AS5600_THREAD_SAFE
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#endif

/** Fixed 7-bit I2C slave address of the AS5600. */
#define AS5600_I2C_ADDRESS 0x36

/** Native 12-bit counts in one complete mechanical revolution (360 degrees). */
#define AS5600_COUNTS_PER_REVOLUTION 4096U

/** Non-volatile zero-position burn count register. */
#define AS5600_REG_ZMCO 0x00

/** High byte of programmed zero position (bits 11:8). */
#define AS5600_REG_ZPOS_H 0x01

/** Low byte of programmed zero position (bits 7:0). */
#define AS5600_REG_ZPOS_L 0x02

/** High byte of programmed maximum position (bits 11:8). */
#define AS5600_REG_MPOS_H 0x03

/** Low byte of programmed maximum position (bits 7:0). */
#define AS5600_REG_MPOS_L 0x04

/** High byte of programmed maximum angle (bits 11:8). */
#define AS5600_REG_MANG_H 0x05

/** Low byte of programmed maximum angle (bits 7:0). */
#define AS5600_REG_MANG_L 0x06

/** High byte of configuration register (WD, FTH, SF). */
#define AS5600_REG_CONF_H 0x07

/** Low byte of configuration register (PWMF, OUTS, HYST, PM). */
#define AS5600_REG_CONF_L 0x08

/** Magnet status flags register. */
#define AS5600_REG_STATUS 0x0B

/** High byte of raw unscaled, unfiltered 12-bit angle (bits 11:8). */
#define AS5600_REG_RAW_ANGLE_H 0x0C

/** Low byte of raw unscaled, unfiltered 12-bit angle (bits 7:0). */
#define AS5600_REG_RAW_ANGLE_L 0x0D

/** High byte of filtered and modified 12-bit angle (bits 11:8). */
#define AS5600_REG_ANGLE_H 0x0E

/** Low byte of filtered and modified 12-bit angle (bits 7:0). */
#define AS5600_REG_ANGLE_L 0x0F

/** Bitmask for Watchdog Timer enable in CONF_H (bit 5 / bit 13). */
#define AS5600_CONF_WD_MASK 0x80

/** Bitmask for Fast Filter Threshold in CONF_H (bits 4:2 / bits 10:8). */
#define AS5600_CONF_FTH_MASK 0x38

/** Bitmask for Slow Filter setting in CONF_H (bits 1:0 / bits 7:6). */
#define AS5600_CONF_SF_MASK 0x06

/** Bitmask for PWM frequency setting in CONF_L (bits 7:6). */
#define AS5600_CONF_PWMF_MASK 0x30

/** Bitmask for Output Stage mode selection in CONF_L (bits 5:4). */
#define AS5600_CONF_OUTS_MASK 0x0C

/** Bitmask for Hysteresis selection in CONF_L (bits 3:2). */
#define AS5600_CONF_HYST_MASK 0x03

/** Status register bit flag indicating a magnet has been detected (MD). */
#define AS5600_STATUS_MD 0x20

/** Status register bit flag indicating magnet field is too weak (ML). */
#define AS5600_STATUS_ML 0x10

/** Status register bit flag indicating magnet field is too strong (MH). */
#define AS5600_STATUS_MH 0x08

/**
 * @brief Runtime filter and interface configuration parameters for the AS5600.
 *
 * Values are programmed into the volatile CONF_H (0x07) and CONF_L (0x08) registers
 * during instance initialization.
 */
typedef struct {
  /**
   * Hysteresis setting (0 = OFF, 1 = 1 LSB, 2 = 2 LSBs, 3 = 3 LSBs).
   * Prevents dithering near quantization thresholds.
   */
  uint8_t hysteresis;

  /**
   * Slow filter factor: 16 (16x), 8 (8x), 4 (4x), or 2 (2x).
   * Lower values reduce step response delay (2x ~286 us; 16x ~2.2 ms).
   * Other values default to 16x in driver initialization.
   */
  uint8_t slow_filter;

  /**
   * Fast filter threshold: 0 to 7 (0 = slow filter only, 1..7 select LSB threshold).
   * When dynamic variation exceeds threshold, internal filter switches to fast tracking.
   */
  uint8_t fast_filter_threshold;

  /**
   * Per-transaction timeout in milliseconds applied to I2C transfers and mutex acquisition.
   * A value of -1 preserves indefinite FreeRTOS waits (portMAX_DELAY). Positive values
   * must convert to at least 1 RTOS tick via pdMS_TO_TICKS().
   */
  int transfer_timeout_ms;
} as5600_config_t;

/**
 * @brief Device context structure representing an active AS5600 instance.
 *
 * Instances are allocated by the application (typically in static or task storage)
 * and must remain valid for the duration of all driver calls.
 */
typedef struct {
  /** ESP-IDF master device handle registered on an initialized I2C bus. */
  i2c_master_dev_handle_t i2c_dev;

  /** Copy of the active configuration parameters written to the sensor. */
  as5600_config_t config;

  /** Effective transaction timeout in milliseconds (-1 for indefinite). */
  int transfer_timeout_ms;

#if CONFIG_AS5600_THREAD_SAFE
  /** Mutex handle serializing concurrent access when thread-safety is enabled. */
  SemaphoreHandle_t lock;

  /** Static buffer for mutex allocation without dynamic heap usage. */
  StaticSemaphore_t lock_buffer;
#endif
} as5600_dev_t;

/**
 * @brief Initialize an AS5600 instance with configuration and verify communication.
 *
 * Writes the specified filter and hysteresis settings to CONF_H and CONF_L,
 * verifies device presence by querying the STATUS register, and configures
 * synchronization mutexes if CONFIG_AS5600_THREAD_SAFE is enabled.
 *
 * If config->transfer_timeout_ms is positive, that timeout is applied to all
 * transactions; otherwise, indefinite waits (-1 / portMAX_DELAY) are retained.
 *
 * Called externally by application code. Internally calls as5600_init_with_timeout().
 *
 * @param[out] dev Pointer to caller-allocated device context structure.
 * @param[in] i2c_dev Initialized ESP-IDF I2C master device handle.
 * @param[in] config Pointer to initialization configuration structure.
 *
 * @return
 *     - ESP_OK: Sensor found and configuration written successfully.
 *     - ESP_ERR_INVALID_ARG: If dev, i2c_dev, or config is NULL.
 *     - ESP_ERR_NO_MEM: If mutex allocation fails.
 *     - ESP_ERR_TIMEOUT: If bus access or mutex times out.
 *     - ESP_FAIL / I2C error: If communication with sensor fails.
 */
esp_err_t as5600_init(as5600_dev_t *dev, i2c_master_dev_handle_t i2c_dev,
                      const as5600_config_t *config);

/**
 * @brief Initialize an AS5600 instance with an explicit transfer timeout.
 *
 * Same lifecycle and configuration behavior as as5600_init(), but explicitly
 * specifies the timeout in milliseconds applied to I2C transfers and mutex locking.
 *
 * Called externally by application code and internally by as5600_init().
 *
 * @param[out] dev Pointer to caller-allocated device context structure.
 * @param[in] i2c_dev Initialized ESP-IDF I2C master device handle.
 * @param[in] config Pointer to initialization configuration structure.
 * @param[in] timeout_ms Timeout in milliseconds (-1 for indefinite, or >= 1 ms).
 *
 * @return
 *     - ESP_OK: Sensor found and configuration written successfully.
 *     - ESP_ERR_INVALID_ARG: If dev, i2c_dev, or config is NULL, or invalid timeout.
 *     - ESP_ERR_NO_MEM: If mutex allocation fails.
 *     - ESP_ERR_TIMEOUT: If bus access or mutex times out.
 *     - ESP_FAIL / I2C error: If communication with sensor fails.
 */
esp_err_t as5600_init_with_timeout(as5600_dev_t *dev,
                                  i2c_master_dev_handle_t i2c_dev,
                                  const as5600_config_t *config,
                                  int timeout_ms);

/**
 * @brief Update the transaction timeout for an existing AS5600 instance.
 *
 * Modifies the per-instance timeout applied to subsequent I2C transactions
 * and mutex acquisition.
 *
 * Called externally by application code.
 *
 * @param[in,out] dev Pointer to initialized device context.
 * @param[in] timeout_ms New timeout in milliseconds (-1 for indefinite, or >= 1 ms).
 *
 * @return
 *     - ESP_OK: Timeout updated.
 *     - ESP_ERR_INVALID_ARG: If dev is NULL or timeout is invalid.
 */
esp_err_t as5600_set_timeout(as5600_dev_t *dev, int timeout_ms);

/**
 * @brief Read the 12-bit unscaled and unfiltered raw angle register (0x0C..0x0D).
 *
 * Reads 2 bytes in a single burst from register 0x0C. Returns the raw CORDIC
 * coordinate before zero-position (ZPOS) and maximum angle (MANG) scaling.
 *
 * Called externally by application acquisition loops.
 *
 * @param[in,out] dev Pointer to initialized device context.
 * @param[out] raw_angle Output pointer receiving the 12-bit angle [0..4095].
 *
 * @return
 *     - ESP_OK: Read successful and angle stored in raw_angle.
 *     - ESP_ERR_INVALID_ARG: If dev or raw_angle is NULL.
 *     - ESP_ERR_TIMEOUT: If mutex or I2C transaction times out.
 *     - ESP_FAIL / I2C error: On peripheral bus failure.
 */
esp_err_t as5600_read_raw_angle(as5600_dev_t *dev, uint16_t *raw_angle);

/**
 * @brief Read the 12-bit filtered and scaled angle register (0x0E..0x0F).
 *
 * Reads 2 bytes in a single burst from register 0x0E. Returns the angle
 * processed through the internal slow/fast filter, hysteresis, and programmed range.
 *
 * Called externally by application code.
 *
 * @param[in,out] dev Pointer to initialized device context.
 * @param[out] angle Output pointer receiving the 12-bit filtered angle [0..4095].
 *
 * @return
 *     - ESP_OK: Read successful and angle stored in angle.
 *     - ESP_ERR_INVALID_ARG: If dev or angle is NULL.
 *     - ESP_ERR_TIMEOUT: If mutex or I2C transaction times out.
 *     - ESP_FAIL / I2C error: On peripheral bus failure.
 */
esp_err_t as5600_read_angle(as5600_dev_t *dev, uint16_t *angle);

/**
 * @brief Read the magnet status register (0x0B).
 *
 * Queries MD (magnet detected), ML (field too low), and MH (field too high) flags.
 *
 * Called externally by application code.
 *
 * @param[in,out] dev Pointer to initialized device context.
 * @param[out] status Output pointer receiving the 8-bit status byte.
 *
 * @return
 *     - ESP_OK: Read successful and status stored.
 *     - ESP_ERR_INVALID_ARG: If dev or status is NULL.
 *     - ESP_ERR_TIMEOUT: If mutex or I2C transaction times out.
 *     - ESP_FAIL / I2C error: On peripheral bus failure.
 */
esp_err_t as5600_read_status(as5600_dev_t *dev, uint8_t *status);

/**
 * @brief Read status, raw angle, and filtered angle in a single sequential query.
 *
 * Performs consecutive register reads to capture diagnostic status and both
 * angle representations. Any NULL parameter skips that specific read.
 *
 * Called externally by application diagnostic tasks.
 *
 * @param[in,out] dev Pointer to initialized device context.
 * @param[out] raw_angle Optional pointer receiving raw angle (may be NULL).
 * @param[out] angle Optional pointer receiving filtered angle (may be NULL).
 * @param[out] status Optional pointer receiving status byte (may be NULL).
 *
 * @return
 *     - ESP_OK: All requested fields read successfully.
 *     - ESP_ERR_INVALID_ARG: If dev is NULL.
 *     - ESP_ERR_TIMEOUT: If mutex or I2C transaction times out.
 *     - ESP_FAIL / I2C error: On peripheral bus failure.
 */
esp_err_t as5600_read_angles_and_status(as5600_dev_t *dev, uint16_t *raw_angle,
                                        uint16_t *angle, uint8_t *status);

#endif /* AS5600_H_ */
