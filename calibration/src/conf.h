/**
 * @file conf.h
 *
 * @defgroup CONF Configurations
 *
 * @brief General configuration of the system.
 *
 */

#ifndef CONF_H

#ifndef F_CPU
#define F_CPU 16000000UL // define a frequencia do microcontrolador - 16MHz
#endif                   /* ifndef F_CPU */

// CONFIGURACOES DE COMPILACAO
// #define DEBUG_ON
// #define VERBOSE_ON
// #define VERBOSE_ON_CAN_APP
// #define VERBOSE_ON_MACHINE
// #define VERBOSE_ON_HMC5883L
// #define VERBOSE_ON_MPU6050
// #define VERBOSE_ON_ADC
// #define VERBOSE_ON_INIT
// #define VERBOSE_ON_ERROR
// #define VERBOSE_ON_HBRIDGE

// MODULES ACTIVATION
#define USART_ON
// #define CAN_ON
// #define CAN_DEPENDENT
// #define HMC5883L_ON
#define MPU9250_ON
// #define MPU6050_ON
// #define ADC_ON
// #define MACHINE_ON
// #define HBRIDGE_ON
// #define LED_ON
#define WATCHDOG_ON
#define WATCHDOG_SETTING WDTO_1S
// #define SLEEP_ON

#define CAN_SIGNATURE_SELF CAN_SIGNATURE_MNA23

// The machine frequency may not be superior of ADC_FREQUENCY/ADC_AVG_SIZE_10
#ifdef MACHINE_ON
#define MACHINE_TIMER_FREQUENCY 300                                               //<! machine timer frequency in Hz
#define MACHINE_TIMER_PRESCALER 1024                                              //<! machine timer prescaler
#define MACHINE_FREQUENCY 100                                                     //<! machine frequency in Hz
#define MACHINE_CLK_DIVIDER_VALUE (MACHINE_TIMER_FREQUENCY) / (MACHINE_FREQUENCY) //<! machine_run clock divider
#endif                                                                            // MACHINE_ON              (MACHINE_TIMER_FREQUENCY)/(MACHINE_CLK_DIVIDER_VALUE)

#ifdef HBRIDGE_ON
// #define HBRIDGE_TIMER_FREQUENCY             0
// #define HBRIDGE_TIMER_PRESCALER             0
// #define HBRIDGE_SWITCHING_FREQUENCY         0
#define HBRIDGE_CLK_DIVIDER_VALUE (HBRIDGE_TIMER_FREQUENCY) / (HBRIDGE_SWITCHING_FREQUENCY)
#define HBRIDGE_PORT PORTD
#define HBRIDGE_PWM_DDR DDRD
#define HBRIDGE_PWM_PIN1 PD6
#define HBRIDGE_PWM_PIN2 PD5
#define HBRIDGE_ENABLE_PIN PD2
#define HBRIDGE_VERBOSE_CLK_DIV 50
#define HBRIDG_SIDE_SWITCH_CLK_DIV 3000
#define TAIL_TOLERANCE 7
#define H_MAX_RANGE 1 / 255
#endif // HBRIDGE_ON

// #### LED PINS ####
#ifdef LED_ON
#define LED1_PORT PORTD
#define LED1_PIN PIND
#define LED1_DDR DDRD
#define LED1 PD3

#define LED2_PORT PORTB
#define LED2_PIN PINB
#define LED2_DDR DDRB
#define LED2 PB2

#else
#define cpl_led()
#define set_led()
#define clr_led()
#endif // LED_ON

#ifdef CAN_ON
#define SPI_ON
#define CAN_APP_SEND_STATE_FREQ 100 // 36000     //<! state msg frequency in Hz
#define CAN_APP_SEND_ADC_FREQ 10    // 6000      //<! adc msg frequency in Hz

// CANBUS DEFINITONS
// ----------------------------------------------------------------------------
/* Global settings for building the can-lib and application program.
 *
 * The following two #defines must be set identically for the can-lib and
 * your application program. They control the underlying CAN struct. If the
 * settings disagree, the underlying CAN struct will be broken, with
 * unpredictable results.
 * If can.h detects that any of the #defines is not defined, it will set them
 * to the default values shown here, so it is in your own interest to have a
 * consistent setting. Ommiting the #defines in both can-lib and application
 * program will apply the defaults in a consistent way too.
 *
 * Select if you want to use 29 bit identifiers.
 */
#define SUPPORT_EXTENDED_CANID 0

/* Select if you want to use timestamps.
 * Timestamps are sourced from a register internal to the AT90CAN.
 * Selecting them on any other controller will have no effect, they will
 * be 0 all the time.
 */
#define SUPPORT_TIMESTAMPS 0
#endif // CAN_ON

#endif /* ifndef CONF_H */
