/*
 * File:   generic_usart.c
 * Author: Gustavo Vianna França
 *
 * Created on April 15, 2023, 1:05 PM
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "Arduino.h"
#include "generic_usart.h"
#include <util/delay_basic.h>

static generic_usart_t *generic_usart_active = NULL;

void generic_usart_init(generic_usart_t *gu, uint32_t speed, uint8_t rx_pin, uint8_t tx_pin, uint16_t inverse_logic, bool disable_pullup)
{
    gu->_buffer_overflow = 0;
    gu->_inverse_logic = inverse_logic;
    generic_usart_set_rx(gu, rx_pin, disable_pullup);
    generic_usart_set_tx(gu, tx_pin, disable_pullup);

    gu->_rx_delay_centering = gu->_rx_delay_intrabit = gu->_rx_delay_stopbit = gu->_tx_delay = 0;

    // Precalculate the various delays, in number of 4-cycle delays
    uint16_t bit_delay = (F_CPU / speed) / 4;

    // 12 (gcc 4.8.2) or 13 (gcc 4.3.2) cycles from start bit to first bit,
    // 15 (gcc 4.8.2) or 16 (gcc 4.3.2) cycles between bits,
    // 12 (gcc 4.8.2) or 14 (gcc 4.3.2) cycles from last bit to stop bit
    // These are all close enough to just use 15 cycles, since the inter-bit
    // timings are the most critical (deviations stack 8 times)
    gu->_tx_delay = SUBTRACT_CAP(bit_delay, 15 / 4);

    // Only setup rx when we have a valid PCINT for this pin
    if (digitalPinToPCICR((int8_t)gu->_receivePin))
    {
#if GCC_VERSION > 40800
        // Timings counted from gcc 4.8.2 output. This works up to 115200 on
        // 16Mhz and 57600 on 8Mhz.
        //
        // When the start bit occurs, there are 3 or 4 cycles before the
        // interrupt flag is set, 4 cycles before the PC is set to the right
        // interrupt vector address and the old PC is pushed on the stack,
        // and then 75 cycles of instructions (including the RJMP in the
        // ISR vector table) until the first delay. After the delay, there
        // are 17 more cycles until the pin value is read (excluding the
        // delay in the loop).
        // We want to have a total delay of 1.5 bit time. Inside the loop,
        // we already wait for 1 bit time - 23 cycles, so here we wait for
        // 0.5 bit time - (71 + 18 - 22) cycles.
        _rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 75 + 17 - 23) / 4);

        // There are 23 cycles in each loop iteration (excluding the delay)
        _rx_delay_intrabit = subtract_cap(bit_delay, 23 / 4);

        // There are 37 cycles from the last bit read to the start of
        // stopbit delay and 11 cycles from the delay until the interrupt
        // mask is enabled again (which _must_ happen during the stopbit).
        // This delay aims at 3/4 of a bit time, meaning the end of the
        // delay will be at 1/4th of the stopbit. This allows some extra
        // time for ISR cleanup, which makes 115200 baud at 16Mhz work more
        // reliably
        _rx_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (37 + 11) / 4);
#else // Timings counted from gcc 4.3.2 output
      // Note that this code is a _lot_ slower, mostly due to bad register
      // allocation choices of gcc. This works up to 57600 on 16Mhz and
      // 38400 on 8Mhz.
        gu->_rx_delay_centering = SUBTRACT_CAP(bit_delay / 2, (4 + 4 + 97 + 29 - 11) / 4);
        gu->_rx_delay_intrabit = SUBTRACT_CAP(bit_delay, 11 / 4);
        gu->_rx_delay_stopbit = SUBTRACT_CAP(bit_delay * 3 / 4, (44 + 17) / 4);
#endif

        // Enable the PCINT for the entire port here, but never disable it
        // (others might also need it, so we disable the interrupt by using
        // the per-pin PCMSK register).
        *digitalPinToPCICR((int8_t)gu->_receivePin) |= _BV(digitalPinToPCICRbit(gu->_receivePin));
        // Precalculate the pcint mask register and value, so setRxIntMask
        // can be used inside the ISR without costing too much time.
        gu->_pcint_maskreg = digitalPinToPCMSK(gu->_receivePin);
        gu->_pcint_maskvalue = _BV(digitalPinToPCMSKbit(gu->_receivePin));

        _delay_loop_2(gu->_tx_delay); // if we were low this establishes the end
    }

    generic_usart_listen(gu);
}

bool generic_usart_listen(generic_usart_t *gu)
{
    if (!gu->_rx_delay_stopbit)
        return false;

    if (generic_usart_active != gu)
    {
        if (generic_usart_active)
            generic_usart_stop_listening(generic_usart_active);

        gu->_buffer_overflow = 0;
        gu->_receive_buffer_head = gu->_receive_buffer_tail = 0;
        generic_usart_active = gu;
        generic_usart_set_int_rx_msk(gu, true);
        return true;
    }

    return false;
}

bool generic_usart_stop_listening(generic_usart_t *gu)
{
    if (generic_usart_active == gu)
    {
        generic_usart_set_int_rx_msk(gu, false);
        generic_usart_active = NULL;
        return true;
    }
    return false;
}

bool generic_usart_is_listening(generic_usart_t *gu)
{
    return gu == generic_usart_active;
}

void generic_usart_set_tx(generic_usart_t *gu, uint8_t pin, bool disable_pullup)
{
    // First write, then set output. If we do this the other way around,
    // the pin would be output low for a short while before switching to
    // output high. Now, it is input with pullup for a short while, which
    // is fine. With inverse logic, either order is fine.
    uint8_t port = 0;

    if (disable_pullup)
        digitalWrite(pin, LOW);
    else
        digitalWrite(pin, gu->_inverse_logic ? LOW : HIGH);

    pinMode(pin, OUTPUT);
    gu->_transmitBitMask = digitalPinToBitMask(pin);
    port = digitalPinToPort(pin);
    gu->_transmitPortRegister = portOutputRegister(port);
}

void generic_usart_set_rx(generic_usart_t *gu, uint8_t pin, bool disable_pullup)
{
    uint8_t port = 0;

    pinMode(pin, INPUT);

    if (disable_pullup)
        digitalWrite(pin, LOW);
    else if (!gu->_inverse_logic)
        digitalWrite(pin, HIGH); // pullup for normal logic!
    gu->_receivePin = pin;
    gu->_receiveBitMask = digitalPinToBitMask(pin);
    port = digitalPinToPort(pin);
    gu->_receivePortRegister = portInputRegister(port);
}

void generic_usart_set_int_rx_msk(generic_usart_t *gu, bool enable)
{
    if (enable)
        *gu->_pcint_maskreg |= gu->_pcint_maskvalue;
    else
        *gu->_pcint_maskreg &= ~gu->_pcint_maskvalue;
}

uint8_t generic_usart_rx_pin_read(generic_usart_t *gu)
{
    return *gu->_receivePortRegister & gu->_receiveBitMask;
}

int generic_usart_read(generic_usart_t *gu)
{
    uint8_t data = 0;

    if (!generic_usart_is_listening(gu))
        return -1;

    // Empty buffer?
    if (gu->_receive_buffer_head == gu->_receive_buffer_tail)
        return -1;

    // Read from "head"
    data = gu->_receive_buffer[gu->_receive_buffer_head]; // grab next byte
    gu->_receive_buffer_head = (gu->_receive_buffer_head + 1) % _GENERIC_USART_MAX_RX_BUFF;
    return data;
}

uint8_t generic_usart_available(generic_usart_t *gu)
{
    if (!generic_usart_is_listening(gu))
        return 0;

    return ((uint8_t)(gu->_receive_buffer_tail + _GENERIC_USART_MAX_RX_BUFF - gu->_receive_buffer_head)) % _GENERIC_USART_MAX_RX_BUFF;
}

uint8_t generic_usart_write(generic_usart_t *gu, uint8_t data)
{
    // By declaring these as local variables, the compiler will put them
    // in registers _before_ disabling interrupts and entering the
    // critical timing sections below, which makes it a lot easier to
    // verify the cycle timings
    volatile uint8_t *reg = gu->_transmitPortRegister;
    uint8_t reg_mask = gu->_transmitBitMask;
    uint8_t inv_mask = ~gu->_transmitBitMask;
    uint8_t oldSREG = SREG;
    bool inv = gu->_inverse_logic;
    uint16_t delay = gu->_tx_delay;
    uint8_t i = 0;

    if (delay == 0)
    {
        return 0;
    }

    if (inv)
        data = ~data;

    cli(); // turn off interrupts for a clean txmit

    // Write the start bit
    if (inv)
        *reg |= reg_mask;
    else
        *reg &= inv_mask;

    _delay_loop_2(delay);

    // Write each of the 8 bits
    for (i = 8; i > 0; --i)
    {
        if (data & 1)         // choose bit
            *reg |= reg_mask; // send 1
        else
            *reg &= inv_mask; // send 0

        _delay_loop_2(delay);
        data >>= 1;
    }

    // restore pin to natural state
    if (inv)
        *reg &= inv_mask;
    else
        *reg |= reg_mask;

    SREG = oldSREG; // turn interrupts back on
    _delay_loop_2(gu->_tx_delay);

    return 1;
}

int generic_usart_peek(generic_usart_t *gu)
{
    if (!generic_usart_is_listening(gu))
        return -1;

    // Empty buffer?
    if (gu->_receive_buffer_head == gu->_receive_buffer_tail)
        return -1;

    // Read from "head"
    return gu->_receive_buffer[gu->_receive_buffer_head];
}

void generic_usart_recv(generic_usart_t *gu)
{
#if GCC_VERSION < 40302
    // Work-around for avr-gcc 4.3.0 OSX version bug
    // Preserve the registers that the compiler misses
    // (courtesy of Arduino forum user *etracer*)
    asm volatile(
        "push r18 \n\t"
        "push r19 \n\t"
        "push r20 \n\t"
        "push r21 \n\t"
        "push r22 \n\t"
        "push r23 \n\t"
        "push r26 \n\t"
        "push r27 \n\t" ::);
#endif

    uint8_t data = 0;
    uint8_t next = 0;
    uint8_t i = 0;

    // If RX line is high, then we don't see any start bit
    // so interrupt is probably not for us
    if (gu->_inverse_logic ? generic_usart_rx_pin_read(gu) : !generic_usart_rx_pin_read(gu))
    {
        // Disable further interrupts during reception, this prevents
        // triggering another interrupt directly after we return, which can
        // cause problems at higher baudrates.
        generic_usart_set_int_rx_msk(gu, false);

        // Wait approximately 1/2 of a bit width to "center" the sample
        _delay_loop_2(gu->_rx_delay_centering);

        // Read each of the 8 bits
        for (i = 8; i > 0; --i)
        {
            _delay_loop_2(gu->_rx_delay_intrabit);
            data >>= 1;
            if (generic_usart_rx_pin_read(gu))
                data |= 0x80;
        }

        if (gu->_inverse_logic)
            data = ~data;

        // if buffer full, set the overflow flag and return
        next = (gu->_receive_buffer_tail + 1) % _GENERIC_USART_MAX_RX_BUFF;
        if (next != gu->_receive_buffer_head)
        {
            // save new data in buffer: tail points to where byte goes
            gu->_receive_buffer[gu->_receive_buffer_tail] = data; // save new byte
            gu->_receive_buffer_tail = next;
        }
        else
        {
            gu->_buffer_overflow = 1;
        }

        // skip the stop bit
        _delay_loop_2(gu->_rx_delay_stopbit);

        // Re-enable interrupts when we're sure to be inside the stop bit
        generic_usart_set_int_rx_msk(gu, true);
    }

#if GCC_VERSION < 40302
    // Work-around for avr-gcc 4.3.0 OSX version bug
    // Restore the registers that the compiler misses
    asm volatile(
        "pop r27 \n\t"
        "pop r26 \n\t"
        "pop r23 \n\t"
        "pop r22 \n\t"
        "pop r21 \n\t"
        "pop r20 \n\t"
        "pop r19 \n\t"
        "pop r18 \n\t" ::);
#endif
}

bool generic_usart_overflow(generic_usart_t *gu)
{
    bool ret = gu->_buffer_overflow;
    if (ret)
        gu->_buffer_overflow = false;
    return ret;
}

void generic_usart_handle_interrupt()
{
    if (generic_usart_active)
    {
        generic_usart_recv(generic_usart_active);
    }
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{
    generic_usart_handle_interrupt();
}
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect));
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect));
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect, ISR_ALIASOF(PCINT0_vect));
#endif

// void generic_usart_enable_pin(uint8_t pin);
// void generic_usart_disable_pin(uint8_t pin);
// void generic_usart_send_char(char data);
// void generic_usart_send_string(const char *data);
// char generic_usart_receive_char(void);