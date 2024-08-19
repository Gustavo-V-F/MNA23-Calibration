#ifndef MPU6050_H
#define MPU6050_H

#define MPU6050_ADDRESS 0x68

#include <avr/io.h>
#include <stdbool.h>

typedef enum
{
    mpu6050_register_self_test_x = 0x0D,
    mpu6050_register_self_test_y,
    mpu6050_register_self_test_z,
    mpu6050_register_self_test_a,
    mpu6050_register_smplrt_div = 0x19,
    mpu6050_register_config,
    mpu6050_register_gyro_config,
    mpu6050_register_accel_config,
    mpu6050_register_ff_thr,    // See https://www.i2cdevlib.com/devices/mpu6050#registers for info on the obscure registers.
    mpu6050_register_ff_dur,    // See https://www.i2cdevlib.com/devices/mpu6050#registers for info on the obscure registers.
    mpu6050_register_mot_thr,   // See https://www.i2cdevlib.com/devices/mpu6050#registers for info on the obscure registers.
    mpu6050_register_mot_dur,   // See https://www.i2cdevlib.com/devices/mpu6050#registers for info on the obscure registers.
    mpu6050_register_zrmot_thr, // See https://www.i2cdevlib.com/devices/mpu6050#registers for info on the obscure registers.
    mpu6050_register_zrmot_dur, // See https://www.i2cdevlib.com/devices/mpu6050#registers for info on the obscure registers.
    mpu6050_register_fifo_en,
    mpu6050_register_int_pin_cfg = 0x37,
    mpu6050_register_int_enable,
    mpu6050_register_dmp_int_status, // See https://www.i2cdevlib.com/devices/mpu6050#registers for info on the obscure registers.
    mpu6050_register_int_status,
    mpu6050_register_accel_xout_h,
    mpu6050_register_accel_xout_l,
    mpu6050_register_accel_yout_h,
    mpu6050_register_accel_yout_l,
    mpu6050_register_accel_zout_h,
    mpu6050_register_accel_zout_l,
    mpu6050_register_temp_out_h,
    mpu6050_register_temp_out_l,
    mpu6050_register_gyro_xout_h,
    mpu6050_register_gyro_xout_l,
    mpu6050_register_gyro_yout_h,
    mpu6050_register_gyro_yout_l,
    mpu6050_register_gyro_zout_h,
    mpu6050_register_gyro_zout_l,
    mpu6050_register_mot_detect_status = 0x61, // See https://www.i2cdevlib.com/devices/mpu6050#registers for info on the obscure registers.
    mpu6050_register_signal_path_reset = 0x68,
    mpu6050_register_mot_detect_ctrl, // See https://www.i2cdevlib.com/devices/mpu6050#registers for info on the obscure registers.
    mpu6050_register_user_ctrl,
    mpu6050_register_pwr_mgmt_1,
    mpu6050_register_pwr_mgmt_2,
    mpu6050_register_bank_sel,       // See https://www.i2cdevlib.com/devices/mpu6050#registers for info on the obscure registers.
    mpu6050_register_mem_start_addr, // See https://www.i2cdevlib.com/devices/mpu6050#registers for info on the obscure registers.
    mpu6050_register_mem_r_w,        // See https://www.i2cdevlib.com/devices/mpu6050#registers for info on the obscure registers.
    mpu6050_register_dmp_cfg_1,      // See https://www.i2cdevlib.com/devices/mpu6050#registers for info on the obscure registers.
    mpu6050_register_dmp_cfg_2,      // See https://www.i2cdevlib.com/devices/mpu6050#registers for info on the obscure registers.
    mpu6050_register_fifo_counth,
    mpu6050_register_fifo_countl,
    mpu6050_register_fifo_r_w,
    mpu6050_register_who_am_i
} mpu6050_reg_t;

typedef enum
{
    mpu6050_fysnc_disabled,
    mpu6050_fysnc_temp_out,
    mpu6050_fysnc_gyro_xout,
    mpu6050_fysnc_gyro_yout,
    mpu6050_fysnc_gyro_zout,
    mpu6050_fysnc_accel_xout,
    mpu6050_fysnc_accel_yout,
    mpu6050_fysnc_accel_zout
} mpu6050_fsync_t;

typedef enum
{
    mpu6050_gyro_range_250deg,  // +/- 250 deg/s (default value)
    mpu6050_gyro_range_500deg,  // +/- 500 deg/s
    mpu6050_gyro_range_1000deg, // +/- 1000 deg/s
    mpu6050_gyro_range_2000deg  // +/- 2000 deg/s
} mpu6050_gyro_range_t;

typedef enum
{
    mpu6050_accel_range_2g, // +/- 2g deg/s (default value)
    mpu6050_accel_range_4g, // +/- 4g deg/s
    mpu6050_accel_range_8g, // +/- 8g deg/s
    mpu6050_accel_range_16g // +/- 16g deg/s
} mpu6050_accel_range_t;

typedef enum
{
    mpu6050_lowpass_filter_disabled, // 260 Hz (filter disabled)
    mpu6050_lowpass_filter_184hz,    // 184 Hz
    mpu6050_lowpass_filter_94hz,     // 94 Hz
    mpu6050_lowpass_filter_44hz,     // 44 Hz
    mpu6050_lowpass_filter_21hz,     // 21 Hz
    mpu6050_lowpass_filter_10hz,     // 10 Hz
    mpu6050_lowpass_filter_5hz       // 5 Hz
} mpu6050_lowpass_filter_t;

typedef enum
{
    mpu6050_highpass_filter_disabled, // Filter disabled
    mpu6050_highpass_filter_5hz,      // 5 Hz
    mpu6050_highpass_filter_2_5hz,    // 2.5 Hz
    mpu6050_highpass_filter_1_25hz,   // 1.25 Hz
    mpu6050_highpass_filter_0_63hz,   // 0.63 Hz
    mpu6050_highpass_filter_hold = 7  // Hold sample
} mpu6050_highpass_filter_t;          // See https://www.i2cdevlib.com/devices/mpu6050#registers for info on the obscure registers.

typedef enum
{
    mpu6050_clk_8mhz,
    mpu6050_clk_pll_gyro_x,
    mpu6050_clk_pll_gyro_y,
    mpu6050_clk_pll_gyro_z,
    mpu6050_clk_pll_ext_32khz,
    mpu6050_clk_pll_ext_19mhz,
    mpu6050_clk_stop = 7
} mpu6050_clk_sel_t;

typedef enum
{
    mpu6050_lp_wake_interval_1_25hz, // 1.25 Hz
    mpu6050_lp_wake_interval_5hz,    // 5 Hz
    mpu6050_lp_wake_interval_20hz,   // 20 Hz
    mpu6050_lp_wake_interval_40hz    // 40 Hz
} mpu6050_lp_wake_ctrl_t;

typedef enum
{
    mpu6050_mask_fsync = 0b00111000,
    mpu6050_mask_lowpass_freq = 0b00000011,
    mpu6050_mask_gyro_range = 0b00011000,
    mpu6050_mask_accel_range = 0b00011000,
    mpu6050_mask_highpass_freq = 0b00000011,
    mpu6050_mask_int_level = 0b10000000,
    mpu6050_mask_int_rd_clear = 0b00010000,
    mpu6050_mask_int_enable = 0b01011001,
    mpu6050_mask_signal_path_reset = 0b00000111,
    mpu6050_mask_sig_cond_reset = 0b00000001,
    mpu6050_mask_sleep = 0b01000000,
    mpu6050_mask_device_reset = 0b10000000,
    mpu6050_mask_clk_sel = 0b00000111,
    mpu6050_mask_wake_up_interval = 0b11000000
} mpu6050_mask_t;

typedef struct
{
    float accel_x; //
    float accel_y; //
    float accel_z; //
    float temp;    //
    float gyro_x;  //
    float gyro_y;  //
    float gyro_z;  //
    uint8_t smplrt_div;
    uint8_t config;
    uint8_t gyro_config;
    uint8_t accel_config;
    uint8_t int_pin_cfg;
    uint8_t int_enable;
    uint8_t pwr_mgmt_1;
    uint8_t pwr_mgmt_2;
    // TODO: add other variables for register values
    uint8_t address;
} mpu6050_data_t;

#ifdef __cplusplus
extern "C"
{
#endif

    void mpu6050_config(mpu6050_data_t *mpu);
    bool mpu6050_ready(mpu6050_data_t *mpu, bool init);
    void mpu6050_write_byte(mpu6050_data_t *mpu, uint8_t reg, uint8_t value);
    uint8_t mpu6050_read_byte(mpu6050_data_t *mpu, uint8_t reg);
    uint8_t mpu6050_read(mpu6050_data_t *mpu);
    void mpu6050_set_addr(mpu6050_data_t *mpu, uint8_t addr);
    void mpu6050_set_smplrt_div(mpu6050_data_t *mpu, uint8_t smplrt);
    void mpu6050_set_fsync(mpu6050_data_t *mpu, mpu6050_fsync_t fsync);
    void mpu6050_set_gyro_range(mpu6050_data_t *mpu, mpu6050_gyro_range_t gyro_range);
    void mpu6050_set_accel_range(mpu6050_data_t *mpu, mpu6050_accel_range_t accel_range);
    void mpu6050_set_lowpass_freq(mpu6050_data_t *mpu, mpu6050_lowpass_filter_t lowpass_freq);
    void mpu6050_set_highpass_freq(mpu6050_data_t *mpu, mpu6050_highpass_filter_t highpass_freq);
    void mpu6050_set_int_level(mpu6050_data_t *mpu, bool enable);
    void mpu6050_set_int_rd_clear(mpu6050_data_t *mpu, bool enable);
    void mpu6050_set_int_enable(mpu6050_data_t *mpu, bool enable);
    void mpu6050_set_signal_path_reset(mpu6050_data_t *mpu, bool rst);
    void mpu6050_set_sig_cond_reset(mpu6050_data_t *mpu, bool rst);
    void mpu6050_set_clk_src(mpu6050_data_t *mpu, mpu6050_clk_sel_t clk_src);
    void mpu6050_set_sleep(mpu6050_data_t *mpu, bool sleep);
    void mpu6050_set_device_reset(mpu6050_data_t *mpu, bool rst);
    void mpu6050_set_wake_up_interval(mpu6050_data_t *mpu, mpu6050_lp_wake_ctrl_t interval);
    uint8_t mpu6050_get_addr(mpu6050_data_t *mpu);
    uint8_t mpu6050_get_smplrt_div(mpu6050_data_t *mpu);
    float mpu6050_get_accel_x(mpu6050_data_t *mpu);
    float mpu6050_get_accel_y(mpu6050_data_t *mpu);
    float mpu6050_get_accel_z(mpu6050_data_t *mpu);
    float mpu6050_get_temp(mpu6050_data_t *mpu);
    float mpu6050_get_gyro_x(mpu6050_data_t *mpu);
    float mpu6050_get_gyro_y(mpu6050_data_t *mpu);
    float mpu6050_get_gyro_z(mpu6050_data_t *mpu);
    mpu6050_fsync_t mpu6050_get_fsync(mpu6050_data_t *mpu);
    mpu6050_gyro_range_t mpu6050_get_gyro_range(mpu6050_data_t *mpu);
    mpu6050_accel_range_t mpu6050_get_accel_range(mpu6050_data_t *mpu);
    mpu6050_lowpass_filter_t mpu6050_get_lowpass_freq(mpu6050_data_t *mpu);
    mpu6050_highpass_filter_t mpu6050_get_highpass_freq(mpu6050_data_t *mpu);
    bool mpu6050_get_int_level(mpu6050_data_t *mpu);
    bool mpu6050_get_int_rd_clear(mpu6050_data_t *mpu);
    bool mpu6050_get_int_enable(mpu6050_data_t *mpu);
    mpu6050_clk_sel_t mpu6050_get_clk_src(mpu6050_data_t *mpu);
    bool mpu6050_get_sleep(mpu6050_data_t *mpu);
    mpu6050_lp_wake_ctrl_t mpu6050_get_wake_up_interval(mpu6050_data_t *mpu);
    // TODO: add dmp initialization procedures, configuration and quaternion calculations.

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */