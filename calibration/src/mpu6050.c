#include <stdbool.h>
#include <util/delay.h>
#include "bit_utils.h"
#include "twi_wire.h"
#include "mpu6050.h"

/// @brief Returns the register value and its modified bits following the mask bit range, the modified bits are set by the chosen setting.
///
/// The @p setting is applied relative to the mask first bit starting from the LSB.
/// Thus the @p mask cannot be a non sequential range of bits (i.e. 0b00110011), in that case the setting will be applied from the beginning of the first range of bits starting from the LSB.
/// Example:
/// @code
/// masked_value = mpu6050_mask_value(&mpu, mpu6050_register_pwr_mgmt_1, mpu6050_clk_pll_gyro_x, mpu6050_mask_clk_sel);
/// @endcode
/// @param mpu The currenty used MPU6050 data address.
/// @param reg Selected MPU6050's register.
/// @param setting The setting's value.
/// @param mask A byte with a sequence of bits or a single bit set in reference to the @p setting.
/// @return Returns the register's masked value.
static uint8_t mpu6050_mask_value(mpu6050_data_t *mpu, uint8_t reg, uint8_t setting, uint8_t mask)
{
    uint8_t reg_value = 0, mask_aux = mask, mask_bitsize = 0;

    for (mask_bitsize = 0; ((!tst_bit(mask_aux, 0)) && (mask_bitsize < 8)); mask_bitsize++)
    {
        mask_aux = mask_aux >> 1;
    }

    reg_value = mpu6050_read_byte(mpu, reg);

    return (reg_value & (~mask)) | ((setting << mask_bitsize) & mask);
}

void mpu6050_config(mpu6050_data_t *mpu)
{
    mpu6050_set_addr(mpu, MPU6050_ADDRESS);
}

bool mpu6050_ready(mpu6050_data_t *mpu, bool init)
{
    if (init)
    {
        twi_wire_leader_init(false);
        mpu->accel_x = 0;
        mpu->accel_y = 0;
        mpu->accel_z = 0;
        mpu->gyro_x = 0;
        mpu->gyro_y = 0;
        mpu->gyro_z = 0;
        mpu->temp = 0;
        mpu6050_set_clk_src(mpu, mpu6050_clk_pll_gyro_x);
        mpu6050_set_gyro_range(mpu, mpu6050_gyro_range_250deg);
        mpu6050_set_accel_range(mpu, mpu6050_accel_range_2g);
        mpu6050_set_sleep(mpu, false);
        // TODO: add dmp initialization.
        // mpu6050_set_device_reset(mpu, true);
        // _delay_ms(100);
        // mpu6050_set_sig_cond_reset(mpu, true);
        // _delay_ms(100);
        // mpu6050_set_clk_src(mpu, mpu6050_clk_pll_gyro_x);
        // mpu6050_set_int_enable(mpu, false);
        // mpu6050_set_accel_range(mpu, mpu6050_accel_range_2g);
        // // mpu6050_set_int_level(mpu, true);
        // // mpu6050_set_int_rd_clear(mpu, true);
        // mpu6050_set_clk_src(mpu, mpu6050_clk_pll_gyro_x);
        // mpu6050_set_smplrt_div(mpu, 0x04);
        // mpu6050_set_lowpass_freq(mpu, mpu6050_lowpass_filter_184hz);
        // mpu6050_set_gyro_range(mpu, mpu6050_gyro_range_2000deg);
        // mpu6050_set_sleep(mpu, false);
        // _delay_ms(100);
    }

    return true;
}

void mpu6050_write_byte(mpu6050_data_t *mpu, uint8_t reg, uint8_t value)
{
    twi_wire_begin_transmission(mpu->address);
    twi_wire_write_byte(reg);
    twi_wire_write_byte(value);
    twi_wire_end_transmission(true);
}

uint8_t mpu6050_read_byte(mpu6050_data_t *mpu, uint8_t reg)
{
    uint8_t value;

    twi_wire_begin_transmission(mpu->address);
    twi_wire_write_byte(reg);
    twi_wire_end_transmission(true);
    twi_wire_request_from(mpu->address, 1, true);

    value = twi_wire_read();

    twi_wire_end_transmission(true);

    return value;
}

uint8_t mpu6050_read(mpu6050_data_t *mpu)
{
    uint8_t i, bytes_read = 0, bytes[TWI_BUFFER_LENGTH] = {0};
    float *data_addr = &(mpu->accel_x);

    twi_wire_begin_transmission(mpu->address);
    twi_wire_write_byte(mpu6050_register_accel_xout_h);
    twi_wire_end_transmission(false);
    bytes_read = twi_wire_request_from(mpu->address, 14, true);

    if (bytes_read > 0)
    {
        for (i = 0; i < bytes_read; i++)
        {
            bytes[i] = twi_wire_read();
        }

        for (i = 0; i < bytes_read; i = i + 2)
        {
            data_addr[i >> 1] = ((int16_t)(bytes[i] << 8) | bytes[i + 1]);
        }
    }

    return bytes_read;
}

void mpu6050_set_addr(mpu6050_data_t *mpu, uint8_t addr)
{
    mpu->address = addr;
}

void mpu6050_set_smplrt_div(mpu6050_data_t *mpu, uint8_t smplrt)
{
    mpu->smplrt_div = smplrt;

    mpu6050_write_byte(mpu, mpu6050_register_smplrt_div, smplrt);
}

void mpu6050_set_fsync(mpu6050_data_t *mpu, mpu6050_fsync_t fsync)
{
    uint8_t reg_setting = 0;

    reg_setting = mpu6050_mask_value(mpu, mpu6050_register_config, fsync, mpu6050_mask_fsync);

    mpu->config = reg_setting;

    mpu6050_write_byte(mpu, mpu6050_register_config, reg_setting);
}

void mpu6050_set_gyro_range(mpu6050_data_t *mpu, mpu6050_gyro_range_t gyro_range)
{
    uint8_t reg_setting = 0;

    reg_setting = mpu6050_mask_value(mpu, mpu6050_register_gyro_config, gyro_range, mpu6050_mask_gyro_range);

    mpu->gyro_config = reg_setting;

    mpu6050_write_byte(mpu, mpu6050_register_gyro_config, reg_setting);
}

void mpu6050_set_accel_range(mpu6050_data_t *mpu, mpu6050_accel_range_t accel_range)
{
    uint8_t reg_setting = 0;

    reg_setting = mpu6050_mask_value(mpu, mpu6050_register_accel_config, accel_range, mpu6050_mask_accel_range);

    mpu->accel_config = reg_setting;

    mpu6050_write_byte(mpu, mpu6050_register_accel_config, reg_setting);
}

void mpu6050_set_lowpass_freq(mpu6050_data_t *mpu, mpu6050_lowpass_filter_t lowpass_freq)
{
    uint8_t reg_setting = 0;

    reg_setting = mpu6050_mask_value(mpu, mpu6050_register_config, lowpass_freq, mpu6050_mask_lowpass_freq);

    mpu->config = reg_setting;

    mpu6050_write_byte(mpu, mpu6050_register_config, reg_setting);
}

void mpu6050_set_highpass_freq(mpu6050_data_t *mpu, mpu6050_highpass_filter_t highpass_freq)
{
    uint8_t reg_setting = 0;

    reg_setting = mpu6050_mask_value(mpu, mpu6050_register_accel_config, highpass_freq, mpu6050_mask_highpass_freq);

    mpu->accel_config = reg_setting;

    mpu6050_write_byte(mpu, mpu6050_register_accel_config, reg_setting);
}

void mpu6050_set_int_level(mpu6050_data_t *mpu, bool enable)
{
}

void mpu6050_set_int_rd_clear(mpu6050_data_t *mpu, bool enable)
{
}

void mpu6050_set_int_enable(mpu6050_data_t *mpu, bool enable)
{
    uint8_t reg_setting = 0;

    reg_setting = mpu6050_mask_value(mpu, mpu6050_register_int_enable, enable ? mpu6050_mask_int_enable : 0, mpu6050_mask_int_enable);

    mpu->int_enable = reg_setting;

    mpu6050_write_byte(mpu, mpu6050_register_int_enable, reg_setting);
}

void mpu6050_set_signal_path_reset(mpu6050_data_t *mpu, bool rst)
{
    uint8_t reg_setting = 0;

    reg_setting = mpu6050_mask_value(mpu, mpu6050_register_signal_path_reset, rst ? mpu6050_mask_signal_path_reset : 0, mpu6050_mask_signal_path_reset);

    mpu6050_write_byte(mpu, mpu6050_register_signal_path_reset, reg_setting);
}

void mpu6050_set_sig_cond_reset(mpu6050_data_t *mpu, bool rst)
{
    uint8_t reg_setting = 0;

    reg_setting = mpu6050_mask_value(mpu, mpu6050_register_user_ctrl, rst, mpu6050_mask_sig_cond_reset);

    mpu6050_write_byte(mpu, mpu6050_register_user_ctrl, reg_setting);
}

void mpu6050_set_clk_src(mpu6050_data_t *mpu, mpu6050_clk_sel_t clk_src)
{
    uint8_t reg_setting = 0;

    reg_setting = mpu6050_mask_value(mpu, mpu6050_register_pwr_mgmt_1, clk_src, mpu6050_mask_clk_sel);

    mpu->pwr_mgmt_1 = reg_setting;

    mpu6050_write_byte(mpu, mpu6050_register_pwr_mgmt_1, reg_setting);
}

void mpu6050_set_sleep(mpu6050_data_t *mpu, bool sleep)
{
    uint8_t reg_setting = 0;

    reg_setting = mpu6050_mask_value(mpu, mpu6050_register_pwr_mgmt_1, (uint8_t)sleep, mpu6050_mask_sleep);

    mpu->pwr_mgmt_1 = reg_setting;

    mpu6050_write_byte(mpu, mpu6050_register_pwr_mgmt_1, reg_setting);
}

void mpu6050_set_device_reset(mpu6050_data_t *mpu, bool rst)
{
    uint8_t reg_setting = 0;

    reg_setting = mpu6050_mask_value(mpu, mpu6050_register_pwr_mgmt_1, (uint8_t)rst, mpu6050_mask_device_reset);

    mpu6050_write_byte(mpu, mpu6050_register_pwr_mgmt_1, reg_setting);
}

void mpu6050_set_wake_up_interval(mpu6050_data_t *mpu, mpu6050_lp_wake_ctrl_t interval)
{
    uint8_t reg_setting = 0;

    reg_setting = mpu6050_mask_value(mpu, mpu6050_register_pwr_mgmt_2, interval, mpu6050_mask_wake_up_interval);

    mpu->pwr_mgmt_2 = reg_setting;

    mpu6050_write_byte(mpu, mpu6050_register_pwr_mgmt_2, reg_setting);
}

uint8_t mpu6050_get_addr(mpu6050_data_t *mpu)
{
    return mpu->address;
}

uint8_t mpu6050_get_smplrt_div(mpu6050_data_t *mpu)
{
    return mpu->smplrt_div;
}

float mpu6050_get_accel_x(mpu6050_data_t *mpu)
{
    return mpu->accel_x;
}

float mpu6050_get_accel_y(mpu6050_data_t *mpu)
{
    return mpu->accel_y;
}

float mpu6050_get_accel_z(mpu6050_data_t *mpu)
{
    return mpu->accel_z;
}

float mpu6050_get_temp(mpu6050_data_t *mpu)
{
    return mpu->temp;
}

float mpu6050_get_gyro_x(mpu6050_data_t *mpu)
{
    return mpu->gyro_x;
}

float mpu6050_get_gyro_y(mpu6050_data_t *mpu)
{
    return mpu->gyro_y;
}

float mpu6050_get_gyro_z(mpu6050_data_t *mpu)
{
    return mpu->gyro_z;
}

mpu6050_fsync_t mpu6050_get_fsync(mpu6050_data_t *mpu)
{
    return (mpu->config & mpu6050_mask_fsync) >> 3;
}

mpu6050_gyro_range_t mpu6050_get_gyro_range(mpu6050_data_t *mpu)
{
    return (mpu->gyro_config & mpu6050_mask_gyro_range) >> 3;
}

mpu6050_accel_range_t mpu6050_get_accel_range(mpu6050_data_t *mpu)
{
    return (mpu->accel_config & mpu6050_mask_accel_range) >> 3;
}

mpu6050_lowpass_filter_t mpu6050_get_lowpass_freq(mpu6050_data_t *mpu)
{
    return (mpu->config & mpu6050_mask_lowpass_freq);
}

mpu6050_highpass_filter_t mpu6050_get_highpass_freq(mpu6050_data_t *mpu)
{
    return (mpu->config & mpu6050_mask_highpass_freq);
}

bool mpu6050_get_int_level(mpu6050_data_t *mpu)
{
    return tst_bit(mpu->int_pin_cfg, 7);
}

bool mpu6050_get_int_rd_clear(mpu6050_data_t *mpu)
{
    return tst_bit(mpu->int_pin_cfg, 4);
}

bool mpu6050_get_int_enable(mpu6050_data_t *mpu)
{
    return (mpu->int_enable && mpu6050_mask_int_enable);
}

mpu6050_clk_sel_t mpu6050_get_clk_src(mpu6050_data_t *mpu)
{
    return (mpu->pwr_mgmt_1 & mpu6050_mask_clk_sel);
}

bool mpu6050_get_sleep(mpu6050_data_t *mpu)
{
    return tst_bit(mpu->pwr_mgmt_1, 6);
}

mpu6050_lp_wake_ctrl_t mpu6050_get_wake_up_interval(mpu6050_data_t *mpu)
{
    return (mpu->pwr_mgmt_2 & mpu6050_mask_wake_up_interval) >> 6;
}
