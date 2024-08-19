#include <stdbool.h>
#include <math.h>
#include "conf.h"
#include "Arduino.h"
#include "bit_utils.h"
#include "twi_wire.h"
#include "hmc5883l.h"

#ifdef DEBUG_ON
#include "usart.h"
#endif

/// @brief Returns the register value and its modified bits following the mask bit range, the modified bits are set by the chosen setting.
///
/// The @p setting is applied relative to the mask first bit starting from the LSB.
/// Thus the @p mask cannot be a non sequential range of bits (i.e. 0b00110011), in that case the setting will be applied from the beginning of the first range of bits starting from the LSB.
/// Example:
/// @code
/// masked_value = hmc5883l_mask_value(&mag, hmc5883l_register_mag_cra_reg_m, hmc5883l_mag_no_bias, hmc5883l_mask_msmt);
/// @endcode
/// @param mag The currenty used HMC5883L data address.
/// @param reg Selected HMC5883L's register.
/// @param setting The setting's value.
/// @param mask A byte with a sequence of bits or a single bit set in reference to the @p setting.
/// @return Returns the register's masked value.
static uint8_t hmc5883l_mask_value(hmc5883l_data_t *mag, uint8_t reg, uint8_t setting, uint8_t mask)
{
    uint8_t reg_value = 0, mask_aux = mask, mask_bitsize = 0;

    for (mask_bitsize = 0; ((!tst_bit(mask_aux, 0)) && (mask_bitsize < 8)); mask_bitsize++)
    {
        mask_aux = mask_aux >> 1;
    }

    reg_value = hmc5883l_read_byte(mag, reg);

    return (reg_value & (~mask)) | ((setting << mask_bitsize) & mask);
}

/// @brief Busy wait for at least the input time (given in milliseconds).
/// Example:
/// @code
/// ready = hmc5883l_busy_wait_ms(6);
/// @endcode
/// @param wait The given time in milliseconds
/// @return Returns true if the wait time has passed, false if the timer is disabled.
static bool hmc5883l_busy_wait_ms(uint32_t wait)
{
    uint32_t start = millis();

    while ((millis() - start) < wait)
    {
    }

    return (millis() != start) ? true : false;
}

void hmc5883l_config(hmc5883l_data_t *mag, float declination, hmc5883l_gain_range_t gain)
{
    hmc5883l_set_addr(mag, HMC5883L_ADDRESS);
    hmc5883l_set_declination(mag, declination);
    mag->gain = gain;
}

bool hmc5883l_ready(hmc5883l_data_t *mag, bool init)
{
    bool ready = false;

    if (init)
    {
        twi_wire_leader_init(false); // Argument is false to disable pullup used in the Arduino Uno board.
        mag->x = 0;
        mag->y = 0;
        mag->z = 0;
        mag->declination = 0;
        mag->heading = 0;
        hmc5883l_set_temp_sensor(mag, true); // Unknown effect in HMC5883L's knockoff. Should enable automatic temperature compensation in HMC5983 (different model from HMC5883L).
        hmc5883l_set_samples_avg(mag, hmc5883l_data_samples_avg_1);
        hmc5883l_set_data_output_rate(mag, hmc5883l_mag_data_output_rate_15);
        hmc5883l_set_measurement_mode(mag, hmc5883l_mag_no_bias);
        hmc5883l_set_gain(mag, mag->gain);
        hmc5883l_set_operating_mode(mag, hmc5883l_mag_single_msmt_mode);
    }
    else
    {
        hmc5883l_set_operating_mode(mag, hmc5883l_mag_single_msmt_mode);
    }

    ready = hmc5883l_busy_wait_ms(6);

#ifdef DEBUG_ON
    if (!ready)
        usart_send_string("HMC5883L HAS ZERO WAIT TIME! TIMER IS PROBABLY DISABLED...");
#endif

    return ready;
}

bool hmc5883l_self_test(hmc5883l_data_t *mag, bool negative)
{
    bool pass = 0, retry = 0;
    hmc5883l_gain_range_t curr_gain = hmc5883l_mag_gain_range_4_7, prev_gain = 0, init_gain = 0;

    mag->x = 0;
    mag->y = 0;
    mag->z = 0;
    mag->declination = 0;
    mag->heading = 0;
    init_gain = hmc5883l_get_gain(mag);

    hmc5883l_set_measurement_mode(mag, negative ? hmc5883l_mag_negative_bias : hmc5883l_mag_positive_bias); // No effect in HMC5883L knockoff. Self test only works in original HMC5883L.
    hmc5883l_set_gain(mag, curr_gain);

    for (curr_gain = hmc5883l_mag_gain_range_4_7; (curr_gain <= hmc5883l_mag_gain_range_8_1) && (!pass); curr_gain += (0x01 << 5))
    {
        do
        {
            hmc5883l_set_operating_mode(mag, hmc5883l_mag_single_msmt_mode);

            hmc5883l_busy_wait_ms(6);

            hmc5883l_read(mag);

            if (curr_gain != prev_gain)
            {
                retry = true;
                prev_gain = curr_gain;
            }
            else
                retry = false;
        } while (retry);

        if ((mag->x < (243 * mag->gauss_2_lsb / 390)) || (mag->x > (575 * mag->gauss_2_lsb / 390)))
        {
            pass = false;
        }
        else if ((mag->y < (243 * mag->gauss_2_lsb / 390)) || (mag->y > (575 * mag->gauss_2_lsb / 390)))
        {
            pass = false;
        }
        else if ((mag->z < (243 * mag->gauss_2_lsb / 390)) || (mag->z > (575 * mag->gauss_2_lsb / 390)))
        {
            pass = false;
        }
        else
            pass = true;
    }

    hmc5883l_set_measurement_mode(mag, hmc5883l_mag_no_bias);
    hmc5883l_set_gain(mag, init_gain);

    hmc5883l_set_operating_mode(mag, hmc5883l_mag_single_msmt_mode);

    hmc5883l_busy_wait_ms(6);

    hmc5883l_read(mag);

    hmc5883l_set_operating_mode(mag, hmc5883l_mag_single_msmt_mode);

    hmc5883l_busy_wait_ms(6);

    return pass;
}

void hmc5883l_write_byte(hmc5883l_data_t *mag, uint8_t reg, uint8_t value)
{
    twi_wire_begin_transmission(mag->address);
    twi_wire_write_byte(reg);
    twi_wire_write_byte(value);
    twi_wire_end_transmission(true);
}

uint8_t hmc5883l_read_byte(hmc5883l_data_t *mag, uint8_t reg)
{
    uint8_t value;

    twi_wire_begin_transmission(mag->address);
    twi_wire_write_byte(reg);
    twi_wire_end_transmission(true);
    twi_wire_request_from(mag->address, 1, true);

    value = twi_wire_read();

    twi_wire_end_transmission(true);

    return value;
}

uint8_t hmc5883l_read(hmc5883l_data_t *mag)
{
    uint8_t x_hi = 0, x_lo = x_hi, z_hi = x_hi, z_lo = x_hi, y_hi = x_hi, y_lo = x_hi, temp_hi = x_hi, temp_lo = x_hi, bytes_read = 0;

    twi_wire_begin_transmission(mag->address);
    twi_wire_write_byte(hmc5883l_register_mag_out_x_h_m);
    twi_wire_end_transmission(false);
    bytes_read = twi_wire_request_from(mag->address, 6, true);

    if (bytes_read == 6)
    {
        x_hi = twi_wire_read();
        x_lo = twi_wire_read();
        z_hi = twi_wire_read();
        z_lo = twi_wire_read();
        y_hi = twi_wire_read();
        y_lo = twi_wire_read();

        mag->x = (int16_t)(x_lo | ((int16_t)x_hi << 8));
        mag->y = (int16_t)(y_lo | ((int16_t)y_hi << 8));
        mag->z = (int16_t)(z_lo | ((int16_t)z_hi << 8));
    }
    else
    {
        mag->x = 0;
        mag->y = 0;
        mag->z = 0;
    }

    twi_wire_begin_transmission(mag->address);
    twi_wire_write_byte(hmc5883l_register_mag_temp_out_h_m);
    twi_wire_end_transmission(false);
    bytes_read += twi_wire_request_from(mag->address, 2, true);

    if (bytes_read == 8)
    {
        temp_hi = twi_wire_read();
        temp_lo = twi_wire_read();

        mag->temp = (int16_t)(temp_lo | ((int16_t)temp_hi << 8));
    }
    else
    {
        mag->temp = 0;
    }

    return bytes_read;
}

void hmc5883l_set_addr(hmc5883l_data_t *mag, uint8_t addr)
{
    mag->address = addr;
}

void hmc5883l_set_declination(hmc5883l_data_t *mag, float declination)
{
    mag->declination = declination;
}

void hmc5883l_set_temp_sensor(hmc5883l_data_t *mag, bool enable)
{
    uint8_t reg_setting = 0;

    reg_setting = hmc5883l_mask_value(mag, hmc5883l_register_mag_cra_reg_m, enable, hmc5883l_mask_temp_sensor);

    hmc5883l_write_byte(mag, hmc5883l_register_mag_cra_reg_m, reg_setting);
}

void hmc5883l_set_samples_avg(hmc5883l_data_t *mag, hmc5883l_samples_avg_t samples)
{
    uint8_t reg_setting = 0;

    reg_setting = hmc5883l_mask_value(mag, hmc5883l_register_mag_cra_reg_m, samples, hmc5883l_mask_samples_avg);

    hmc5883l_write_byte(mag, hmc5883l_register_mag_cra_reg_m, reg_setting);
}

void hmc5883l_set_data_output_rate(hmc5883l_data_t *mag, hmc5883l_data_output_rate_t rate)
{
    uint8_t reg_setting = 0;

    reg_setting = hmc5883l_mask_value(mag, hmc5883l_register_mag_cra_reg_m, rate, hmc5883l_mask_data_output_rate);

    hmc5883l_write_byte(mag, hmc5883l_register_mag_cra_reg_m, reg_setting);
}

void hmc5883l_set_measurement_mode(hmc5883l_data_t *mag, hmc5883l_measurement_mode_t mode)
{
    uint8_t reg_setting = 0;

    reg_setting = hmc5883l_mask_value(mag, hmc5883l_register_mag_cra_reg_m, mode, hmc5883l_mask_measurement_mode);

    hmc5883l_write_byte(mag, hmc5883l_register_mag_cra_reg_m, reg_setting);
}

void hmc5883l_set_operating_mode(hmc5883l_data_t *mag, hmc5883l_operating_mode_t mode)
{
    uint8_t reg_setting = 0;

    reg_setting = hmc5883l_mask_value(mag, hmc5883l_register_mag_mr_reg_m, mode, hmc5883l_mask_operating_mode);

    hmc5883l_write_byte(mag, hmc5883l_register_mag_mr_reg_m, reg_setting);
}

void hmc5883l_set_gain(hmc5883l_data_t *mag, hmc5883l_gain_range_t gain)
{
    hmc5883l_write_byte(mag, hmc5883l_register_mag_crb_reg_m, gain);

    mag->gain = gain;

    switch (gain)
    {
    case hmc5883l_mag_gain_range_0_88:
        mag->gauss_2_lsb = 1370;
        break;
    case hmc5883l_mag_gain_range_1_3:
        mag->gauss_2_lsb = 1090;
        break;
    case hmc5883l_mag_gain_range_1_9:
        mag->gauss_2_lsb = 820;
        break;
    case hmc5883l_mag_gain_range_2_5:
        mag->gauss_2_lsb = 660;
        break;
    case hmc5883l_mag_gain_range_4_0:
        mag->gauss_2_lsb = 440;
        break;
    case hmc5883l_mag_gain_range_4_7:
        mag->gauss_2_lsb = 390;
        break;
    case hmc5883l_mag_gain_range_5_6:
        mag->gauss_2_lsb = 330;
        break;
    case hmc5883l_mag_gain_range_8_1:
        mag->gauss_2_lsb = 230;
        break;
    }
}

uint8_t hmc5883l_get_addr(hmc5883l_data_t *mag)
{
    return mag->address;
}

float hmc5883l_get_x(hmc5883l_data_t *mag)
{
    return mag->x;
}

float hmc5883l_get_y(hmc5883l_data_t *mag)
{
    return mag->y;
}

float hmc5883l_get_z(hmc5883l_data_t *mag)
{
    return mag->z;
}

float hmc5883l_get_temp(hmc5883l_data_t *mag)
{
    return mag->temp;
}

float hmc5883l_get_heading_rad(hmc5883l_data_t *mag)
{
    mag->heading = atan2(mag->y, mag->x);
    mag->heading += mag->declination;

    if (mag->heading < 0)
    {
        mag->heading += 2 * M_PI;
    }

    if (mag->heading > 2 * M_PI)
    {
        mag->heading -= 2 * M_PI;
    }

    return mag->heading;
}

float hmc5883l_get_heading_deg(hmc5883l_data_t *mag)
{
    return hmc5883l_get_heading_rad(mag) * 180 / M_PI;
}

float hmc5883l_get_declination(hmc5883l_data_t *mag)
{
    return mag->declination;
}

hmc5883l_gain_range_t hmc5883l_get_gain(hmc5883l_data_t *mag)
{
    return mag->gain;
}
