/**
 * @file dbg_vrb.h
 *
 * @defgroup DBG_VRB    Debug And Verbose
 *
 * @brief Debug and Verbose mode definitions
 *
 */

#ifndef DBG_VRB_H
#define DBG_VRB_H

#ifdef VERBOSE_ON
#define VERBOSE_MSG(x) x
#else
#define VERBOSE_MSG(x)
#endif

#ifdef VERBOSE_ON_ERROR
#define VERBOSE_MSG_ERROR(x) x
#else
#define VERBOSE_MSG_ERROR(x)
#endif

#ifdef VERBOSE_ON_CAN_APP
#define VERBOSE_MSG_CAN_APP(x) x
#else
#define VERBOSE_MSG_CAN_APP(x)
#endif

#ifdef VERBOSE_ON_HMC5883L
#define VERBOSE_MSG_HMC5883L(x) x
#else
#define VERBOSE_MSG_HMC5883L(x)
#endif

#ifdef VERBOSE_ON_MPU6050
#define VERBOSE_MSG_MPU6050(x) x
#else
#define VERBOSE_MSG_MPU6050(x)
#endif

#ifdef VERBOSE_ON_MPU9250
#define VERBOSE_MSG_MPU9250(x) x
#else
#define VERBOSE_MSG_MPU9250(x)
#endif

#ifdef VERBOSE_ON_ADC
#define VERBOSE_MSG_ADC(x) x
#else
#define VERBOSE_MSG_ADC(x)
#endif

#ifdef VERBOSE_ON_PWM
#define VERBOSE_MSG_PWM(x) x
#else
#define VERBOSE_MSG_PWM(x)
#endif

#ifdef VERBOSE_ON_INIT
#define VERBOSE_MSG_INIT(x) x
#else
#define VERBOSE_MSG_INIT(x)
#endif

#ifdef VERBOSE_ON_MACHINE
#define VERBOSE_MSG_MACHINE(x) x
#else
#define VERBOSE_MSG_MACHINE(x)
#endif

#ifdef VERBOSE_ON_HBRIDGE
#define VERBOSE_MSG_HBRIDGE(x) x
#else
#define VERBOSE_MSG_HBRIDGE(x)
#endif

#ifdef DEBUG_ON
#define DEBUG_MSG(x) x
// #define DEBUG0 cpl_bit(PORTB, PB2);
// #define DEBUG1 cpl_bit(PORTB, PB2);
#define DEBUG0
#define DEBUG1
#else
#define DEBUG_MSG(x)
#define DEBUG0
#define DEBUG1
#endif

// https://stackoverflow.com/a/10791845/3850957
// usage:
// #pragma message "The value of ABC: " XSTR(ABC)
#define XSTR(x) STR(x)
#define STR(x) #x

#endif /* ifndef DBG_VRB_H */
