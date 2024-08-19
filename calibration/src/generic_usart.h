/*
 * File:   generic_usart.h
 * Author: Gustavo Vianna França
 *
 * Created on April 15, 2023, 1:02 PM
 */

#ifndef GENERIC_USART_H
#define GENERIC_USART_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <avr/io.h>
#include <stdbool.h>

#ifndef F_CPU
#define F_CPU 16000000UL // define a frequencia do microcontrolador - 16MHz
#endif                   /* ifndef F_CPU */

#ifndef ARDUINO_MAIN
#define ARDUINO_MAIN // Port mapping for Arduino Uno
#endif

#ifndef GENERIC_USART_BAUD
#define GENERIC_USART_BAUD 57600 // Standard baud rate
#endif

#ifndef _GENERIC_USART_MAX_RX_BUFF
#define _GENERIC_USART_MAX_RX_BUFF 64 // RX buffer size
#endif

#define SUBTRACT_CAP(num, sub) ((uint16_t)((num > sub ? num - sub : 1)))

    typedef struct
    {
        uint8_t _receivePin;
        uint8_t _receiveBitMask;
        volatile uint8_t *_receivePortRegister;
        uint8_t _transmitBitMask;
        volatile uint8_t *_transmitPortRegister;
        volatile uint8_t *_pcint_maskreg;
        uint8_t _pcint_maskvalue;

        // Expressed as 4-cycle delays (must never be 0!)
        uint16_t _rx_delay_centering;
        uint16_t _rx_delay_intrabit;
        uint16_t _rx_delay_stopbit;
        uint16_t _tx_delay;

        uint16_t _buffer_overflow;
        uint16_t _inverse_logic;

        uint8_t _receive_buffer[_GENERIC_USART_MAX_RX_BUFF];
        volatile uint8_t _receive_buffer_tail;
        volatile uint8_t _receive_buffer_head;
    } generic_usart_t;

    void generic_usart_init(generic_usart_t *gu, uint32_t speed, uint8_t rx_pin, uint8_t tx_pin, uint16_t inverse_logic, bool disable_pullup);
    bool generic_usart_listen(generic_usart_t *gu);
    bool generic_usart_stop_listening(generic_usart_t *gu);
    bool generic_usart_is_listening(generic_usart_t *gu);
    void generic_usart_set_tx(generic_usart_t *gu, uint8_t pin, bool disable_pullup);
    void generic_usart_set_rx(generic_usart_t *gu, uint8_t pin, bool disable_pullup);
    void generic_usart_set_int_rx_msk(generic_usart_t *gu, bool enable);
    uint8_t generic_usart_rx_pin_read(generic_usart_t *gu);
    int generic_usart_read(generic_usart_t *gu);
    uint8_t generic_usart_available(generic_usart_t *gu);
    uint8_t generic_usart_write(generic_usart_t *gu, uint8_t data);
    int generic_usart_peek(generic_usart_t *gu);
    void generic_usart_recv(generic_usart_t *gu);
    bool generic_usart_overflow(generic_usart_t *gu);
    void generic_usart_handle_interrupt(void);
    //    void generic_usart_enable_pin(uint8_t pin);
    //    void generic_usart_disable_pin(uint8_t pin);
    //    void generic_usart_send_char(char data);
    //    void generic_usart_send_string(const char *data);
    //    char generic_usart_receive_char(void);

#ifdef __cplusplus
}
#endif

#endif /* GENERIC_USART_H */
