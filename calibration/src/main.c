/*
 * File:   main.c
 * Author: Gustavo Vianna França
 *
 * Created on April 15, 2023, 3:19 PM
 */
#ifndef USART_BAUD
#define USART_BAUD 57600
#endif

#include "conf.h"
#include "dbg_vrb.h"
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Arduino.h"
#include "dbg_vrb.h"
#include "usart.h"
#include "generic_usart.h"
// #include "hmc5883l.h"
// #include "mpu6050.h"
#include "mpu9250.h"
#include "watchdog.h"

int main(void)
{

#ifdef MPU9250_ON
    mpu9250_data_t mpu;
#endif

#ifdef MPU6050_ON
    mpu6050_data_t mpu;
#endif

#ifdef HMC5883L_ON
    uint32_t start = 0;
#endif

#ifdef HMC5883L_ON
    hmc5883l_data_t mag;
    hmc5883l_gain_range_t gain = hmc5883l_mag_gain_range_1_3;
    float declination = -0.41015; // Location dependent
    hmc5883l_config(&mag, declination, gain);
#endif

#ifdef MPU9250_ON
    bool available_mpu = false;
    bool init_mpu = true;
#endif

#ifdef MPU6050_ON
    bool available_mpu = 0;
    bool init_mpu = true;
#endif

#ifdef HMC5883L_ON
    bool available_mag = 0;
    bool init_mag = true;
#endif

#ifdef USART_ON
    VERBOSE_MSG_INIT(usart_send_string("USART..."));
    usart_init(MYUBRR, 1, 1); // inicializa a usart
    VERBOSE_MSG_INIT(usart_send_string(" OK!\n"));
#endif

    _delay_ms(200);

#ifdef WATCHDOG_ON
    VERBOSE_MSG_INIT(usart_send_string("WATCHDOG..."));
    wdt_init();
    VERBOSE_MSG_INIT(usart_send_string(" OK!\n"));
    wdt_reset();
#else
    VERBOSE_MSG_INIT(usart_send_string("WATCHDOG... OFF!\n"));
#endif

#ifdef WATCHDOG_ON
    wdt_reset();
#endif

    init();

#ifdef WATCHDOG_ON
    wdt_reset();
#endif

#ifdef MPU9250_ON
    mpu9250_config(&mpu);
#endif

#ifdef MPU6050_ON
    mpu6050_config(&mpu);
#endif

#ifdef WATCHDOG_ON
    wdt_reset();
#endif

#ifdef HMC5883L_ON
    hmc5883l_config(&mag, declination, gain);
#endif

#ifdef WATCHDOG_ON
    wdt_reset();
#endif

    sei();

#ifdef WATCHDOG_ON
    wdt_reset();
#endif

    while (true)
    {
#ifdef MPU9250_ON
        available_mpu = mpu9250_ready(&mpu, init_mpu);
        init_mpu = false;

        if (available_mpu)
        {
            if (mpu9250_read(&mpu))
            {
                usart_send_string("\nRaw:");
                usart_send_float(mpu9250_get_accel_x(&mpu));
                usart_send_string(",");
                usart_send_float(mpu9250_get_accel_y(&mpu));
                usart_send_string(",");
                usart_send_float(mpu9250_get_accel_z(&mpu));
                usart_send_string(",");
                usart_send_float(mpu9250_get_gyro_x(&mpu));
                usart_send_string(",");
                usart_send_float(mpu9250_get_gyro_y(&mpu));
                usart_send_string(",");
                usart_send_float(mpu9250_get_gyro_z(&mpu));

#ifdef WATCHDOG_ON
                wdt_reset();
#endif
            }

            if (mpu9250_read_mag(&mpu))
            {
                usart_send_string(",");
                usart_send_float(mpu9250_get_mag_x(&mpu));
                usart_send_string(",");
                usart_send_float(mpu9250_get_mag_y(&mpu));
                usart_send_string(",");
                usart_send_float(mpu9250_get_mag_z(&mpu));
#ifdef WATCHDOG_ON
                wdt_reset();
#endif
            }

            usart_send_string("\nUni:");
            usart_send_float((mpu9250_get_accel_x(&mpu) / 16384) * 9.80665);
            usart_send_string(",");
            usart_send_float((mpu9250_get_accel_y(&mpu) / 16384) * 9.80665);
            usart_send_string(",");
            usart_send_float((mpu9250_get_accel_z(&mpu) / 16384) * 9.80665);
            usart_send_string(",");
            usart_send_float((mpu9250_get_gyro_x(&mpu) / (32768 / 250)) * 0.017453293F);
            usart_send_string(",");
            usart_send_float((mpu9250_get_gyro_y(&mpu) / (32768 / 250)) * 0.017453293F);
            usart_send_string(",");
            usart_send_float((mpu9250_get_gyro_z(&mpu) / (32768 / 250)) * 0.017453293F);
            usart_send_string(",");
            usart_send_float((mpu9250_get_mag_x(&mpu)));
            usart_send_string(",");
            usart_send_float((mpu9250_get_mag_y(&mpu)));
            usart_send_string(",");
            usart_send_float((mpu9250_get_mag_z(&mpu)));
        }
#endif

#ifdef MPU6050_ON
        available_mpu = mpu6050_ready(&mpu, init_mpu);
        init_mpu = false;

        if (available_mpu)
        {
            if (mpu6050_read(&mpu))
            {
                usart_send_string("Raw:");
                usart_send_float(mpu6050_get_accel_x(&mpu));
                usart_send_string(",");
                usart_send_float(mpu6050_get_accel_y(&mpu));
                usart_send_string(",");
                usart_send_float(mpu6050_get_accel_z(&mpu));
                usart_send_string(",");
                usart_send_float(mpu6050_get_gyro_x(&mpu));
                usart_send_string(",");
                usart_send_float(mpu6050_get_gyro_y(&mpu));
                usart_send_string(",");
                usart_send_float(mpu6050_get_gyro_z(&mpu));

#ifdef WATCHDOG_ON
                wdt_reset();
#endif
            }
        }
#endif

#ifdef HMC5883L_ON
        available_mag = hmc5883l_ready(&mag, init_mag);
        if (init_mag)
            hmc5883l_config(&mag, declination, gain);
        init_mag = false;

#ifdef HMC5883L_ON
#ifdef MPU6050_ON
        VERBOSE_MSG_HMC5883L(usart_send_string("\n\n"));
#endif
        VERBOSE_MSG_HMC5883L(usart_send_string("HMC5883L READY?..."));
        if (available_mag)
            VERBOSE_MSG_HMC5883L(usart_send_string("YES!\n"));
        else
            VERBOSE_MSG_HMC5883L(usart_send_string("NO.\n"));

#endif
        if (available_mag)
        {
            if (hmc5883l_read(&mag))
            {
#ifndef MPU6050_ON
                usart_send_string("Raw:");
                usart_send_float(0.0);
                usart_send_string(",");
                usart_send_float(0.0);
                usart_send_string(",");
                usart_send_float(0.0);
                usart_send_string(",");
                usart_send_float(0.0);
                usart_send_string(",");
                usart_send_float(0.0);
                usart_send_string(",");
                usart_send_float(0.0);
#endif
                // usart_send_string(",");
                // usart_send_float((float)hmc5883l_get_x(&mag));
                // usart_send_string(",");
                // usart_send_float((float)hmc5883l_get_y(&mag));
                // usart_send_string(",");
                // usart_send_float((float)hmc5883l_get_z(&mag));
                // usart_send_string("\n");

#ifdef HMC5883L_ON
                if ((start == 0) || ((millis() - start) > 1000))
                {
                    VERBOSE_MSG_HMC5883L(usart_send_string("\nHMC5883L TEMPERATURE:"));
                    VERBOSE_MSG_HMC5883L(usart_send_float(((highByte((uint16_t)hmc5883l_get_temp(&mag)) * (2 ^ 8) + lowByte((uint16_t)hmc5883l_get_temp(&mag))) / ((2 ^ 4) * 8)) + 25));
                    VERBOSE_MSG_HMC5883L(usart_send_string("\n\n"));

                    start = millis();
                }
#endif
#ifdef WATCHDOG_ON
                wdt_reset();
#endif
            }
        }
#endif

#if defined MPU6050_ON || defined HMC5883L_ON

#if defined MPU6050_ON && defined HMC5883L_ON
        if (!init_mpu && !init_mag)
#elif defined MPU6050_ON
        if (!init_mpu)
#else
        if (!init_mag)
#endif
        {
            usart_send_string("Uni:");
#ifdef MPU6050_ON
            usart_send_float((mpu6050_get_accel_x(&mpu) / 16384) * 9.80665);
            usart_send_string(",");
            usart_send_float((mpu6050_get_accel_y(&mpu) / 16384) * 9.80665);
            usart_send_string(",");
            usart_send_float((mpu6050_get_accel_z(&mpu) / 16384) * 9.80665);
            usart_send_string(",");
            usart_send_float((mpu6050_get_gyro_x(&mpu) / (32768 / 250)) * 0.017453293F);
            usart_send_string(",");
            usart_send_float((mpu6050_get_gyro_y(&mpu) / (32768 / 250)) * 0.017453293F);
            usart_send_string(",");
            usart_send_float((mpu6050_get_gyro_z(&mpu) / (32768 / 250)) * 0.017453293F);
#else
            usart_send_float(0.0);
            usart_send_string(",");
            usart_send_float(0.0);
            usart_send_string(",");
            usart_send_float(0.0);
            usart_send_string(",");
            usart_send_float(0.0);
            usart_send_string(",");
            usart_send_float(0.0);
            usart_send_string(",");
            usart_send_float(0.0);
#endif

#ifdef HMC5883L_ON
            // usart_send_string(",");
            usart_send_float((hmc5883l_get_x(&mag) * (1000 / mag.gauss_2_lsb)) / 10);
            usart_send_string(",");
            usart_send_float((hmc5883l_get_y(&mag) * (1000 / mag.gauss_2_lsb)) / 10);
            usart_send_string(",");
            usart_send_float((hmc5883l_get_z(&mag) * (1000 / mag.gauss_2_lsb)) / 10);
            // usart_send_string("\nGraus:");
            // usart_send_float(hmc5883l_get_heading_deg(&mag));
            // usart_send_string("º,");
            // usart_send_float(hmc5883l_get_heading_rad(&mag));
            // usart_send_string(",");
            // usart_send_float(hmc5883l_get_declination(&mag));
            // usart_send_string(",");
            // usart_send_float(mag.declination);
            usart_send_string("\n");

#else
            usart_send_string(",");
            usart_send_float(0.0);
            usart_send_string(",");
            usart_send_float(0.0);
            usart_send_string(",");
            usart_send_float(0.0);
            usart_send_string("\n");
#endif

#ifdef WATCHDOG_ON
            wdt_reset();
#endif
        }
#endif
    }
}

ISR(BADISR_vect)
{
    while (true)
    {
        VERBOSE_MSG_ERROR(usart_send_string("\nFATAL ERROR: BAD ISR."));
#ifdef WATCHDOG_ON
        VERBOSE_MSG_ERROR(usart_send_string("WAITING FOR WATCHDOG TO RESET...\n"));
#endif
        // #ifdef DEBUG_ON
        //         DEBUG0;
        //         DEBUG1;
        //         _delay_ms(100);
        // #endif
    }
}