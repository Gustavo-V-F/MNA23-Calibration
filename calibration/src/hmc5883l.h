/*
 * File:   hmc5883l.h
 * Author: Gustavo Vianna França
 *
 * Created on April 16, 2023, 8:59 PM
 */

#ifndef HMC5883L_H
#define HMC5883L_H

#define HMC5883L_ADDRESS 0x1E

#include <stdbool.h>

typedef enum
{
  hmc5883l_register_mag_cra_reg_m = 0x00,
  hmc5883l_register_mag_crb_reg_m = 0x01,
  hmc5883l_register_mag_mr_reg_m = 0x02,
  hmc5883l_register_mag_out_x_h_m = 0x03,
  hmc5883l_register_mag_out_x_l_m = 0x04,
  hmc5883l_register_mag_out_z_h_m = 0x05,
  hmc5883l_register_mag_out_z_l_m = 0x06,
  hmc5883l_register_mag_out_y_h_m = 0x07,
  hmc5883l_register_mag_out_y_l_m = 0x08,
  hmc5883l_register_mag_sr_reg_mg = 0x09, // Status register doesn't work on knockoff HMC5883L
  hmc5883l_register_mag_ira_reg_m = 0x0a,
  hmc5883l_register_mag_irb_reg_m = 0x0b,
  hmc5883l_register_mag_irc_reg_m = 0x0c,
  hmc5883l_register_mag_temp_out_h_m = 0x31, // Temperature sensor should only be present in HMC5983 (different model from HMC5883L). HMC5883L's knockoff seem to have it but is uncalibrated.
  hmc5883l_register_mag_temp_out_l_m = 0x32
} hmc5883l_reg_t;

typedef enum
{
  hmc5883l_mask_temp_sensor = 0b10000000,
  hmc5883l_mask_samples_avg = 0b01100000,
  hmc5883l_mask_data_output_rate = 0b00011100,
  hmc5883l_mask_measurement_mode = 0b00000011,
  hmc5883l_mask_rdy = 0b00000001,
  hmc5883l_mask_operating_mode = 0b00000011
} hmc5883l_mask_t;

typedef enum
{
  hmc5883l_data_samples_avg_1 = 0x00, // 1 sample (default)
  hmc5883l_data_samples_avg_2,        // 2 samples
  hmc5883l_data_samples_avg_4,        // 4 samples
  hmc5883l_data_samples_avg_8         // 8 samples
} hmc5883l_samples_avg_t;

typedef enum
{
  hmc5883l_mag_data_output_rate_0_75 = 0x00, // 0.75 Hz
  hmc5883l_mag_data_output_rate_1_5,         // 1.5 Hz
  hmc5883l_mag_data_output_rate_3,           // 3 Hz
  hmc5883l_mag_data_output_rate_7_5,         // 7.5 Hz
  hmc5883l_mag_data_output_rate_15,          // 15 Hz (default)
  hmc5883l_mag_data_output_rate_30,          // 30 Hz
  hmc5883l_mag_data_output_rate_75           // 75 Hz
} hmc5883l_data_output_rate_t;

typedef enum
{
  hmc5883l_mag_no_bias = 0x00, // Normal measurement mode (default)
  hmc5883l_mag_positive_bias,  // Positive bias applied to x,y and z measurements
  hmc5883l_mag_negative_bias   // Negative bias applied to x,y and z measurements
} hmc5883l_measurement_mode_t;

typedef enum
{
  hmc5883l_mag_gain_range_0_88 = 0x10, // +/- 0.88
  hmc5883l_mag_gain_range_1_3 = 0x20,  // +/- 1.3 (default)
  hmc5883l_mag_gain_range_1_9 = 0x40,  // +/- 1.9
  hmc5883l_mag_gain_range_2_5 = 0x60,  // +/- 2.5
  hmc5883l_mag_gain_range_4_0 = 0x80,  // +/- 4.0
  hmc5883l_mag_gain_range_4_7 = 0xa0,  // +/- 4.7
  hmc5883l_mag_gain_range_5_6 = 0xc0,  // +/- 5.6
  hmc5883l_mag_gain_range_8_1 = 0xe0   // +/- 8.1
} hmc5883l_gain_range_t;

typedef enum
{
  hmc5883l_mag_continuous_msmt_mode = 0x00,
  hmc5883l_mag_single_msmt_mode,
  hmc5883l_mag_idle_mode
} hmc5883l_operating_mode_t;

typedef struct
{
  float x;           // Magnetometer x value
  float y;           // Magnetometer y value
  float z;           // Magnetometer z value
  float temp;        // Relative temperature
  float heading;     // Magnetometer heading
  float declination; // Declination provided by GNSS device
  hmc5883l_gain_range_t gain;
  float gauss_2_lsb;
  uint8_t address;
} hmc5883l_data_t;

#ifdef __cplusplus
extern "C"
{
#endif

  void hmc5883l_config(hmc5883l_data_t *mag, float declination, hmc5883l_gain_range_t gain);
  bool hmc5883l_ready(hmc5883l_data_t *mag, bool init);
  bool hmc5883l_self_test(hmc5883l_data_t *mag, bool negative);
  void hmc5883l_write_byte(hmc5883l_data_t *mag, uint8_t reg, uint8_t value);
  uint8_t hmc5883l_read_byte(hmc5883l_data_t *mag, uint8_t reg);
  uint8_t hmc5883l_read(hmc5883l_data_t *mag);
  void hmc5883l_set_addr(hmc5883l_data_t *mag, uint8_t addr);
  void hmc5883l_set_declination(hmc5883l_data_t *mag, float declination);
  void hmc5883l_set_temp_sensor(hmc5883l_data_t *mag, bool enable);
  void hmc5883l_set_samples_avg(hmc5883l_data_t *mag, hmc5883l_samples_avg_t samples);
  void hmc5883l_set_data_output_rate(hmc5883l_data_t *mag, hmc5883l_data_output_rate_t rate);
  void hmc5883l_set_measurement_mode(hmc5883l_data_t *mag, hmc5883l_measurement_mode_t mode);
  void hmc5883l_set_operating_mode(hmc5883l_data_t *mag, hmc5883l_operating_mode_t mode);
  void hmc5883l_set_gain(hmc5883l_data_t *mag, hmc5883l_gain_range_t gain);
  uint8_t hmc5883l_get_addr(hmc5883l_data_t *mag);
  float hmc5883l_get_x(hmc5883l_data_t *mag);
  float hmc5883l_get_y(hmc5883l_data_t *mag);
  float hmc5883l_get_z(hmc5883l_data_t *mag);
  float hmc5883l_get_temp(hmc5883l_data_t * mag);
  float hmc5883l_get_heading_rad(hmc5883l_data_t *mag);
  float hmc5883l_get_heading_deg(hmc5883l_data_t *mag);
  float hmc5883l_get_declination(hmc5883l_data_t *mag);
  hmc5883l_gain_range_t hmc5883l_get_gain(hmc5883l_data_t *mag);

#ifdef __cplusplus
}
#endif

#endif /* HMC5883L_H */
