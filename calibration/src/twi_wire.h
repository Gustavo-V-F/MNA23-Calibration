/* 
 * File:   twi_wire.h
 * Author: Gustavo Vianna França
 *
 * Created on April 16, 2023, 8:59 PM
 */

#ifndef TWI_WIRE_H
#define TWI_WIRE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include "twi.h"

#define TWI_WIRE_BUFFER_LENGTH 32
    void twi_wire_leader_init(bool pullup);
    void twi_wire_follower_init(uint8_t address, bool pullup);
    void twi_wire_disable(void);
    void twi_wire_frequency(uint32_t freq);
    bool twi_wire_get_timeout_flag(void);
    void twi_wire_clear_timeout_flag(void);
    void twi_wire_timeout(uint32_t timeout, bool reset_with_timeout);
    uint8_t twi_wire_request_with_reg_from(
            uint8_t address,
            uint8_t size,
            uint32_t internal_address,
            uint8_t internal_size,
            uint8_t send_stop);
    uint8_t twi_wire_request_from(uint8_t address, uint8_t size, uint8_t send_stop);
    void twi_wire_begin_transmission(uint8_t address);
    uint8_t twi_wire_end_transmission(uint8_t send_stop);
    size_t twi_wire_write_byte(uint8_t data);
    size_t twi_wire_write(const uint8_t *data, size_t size);
    uint8_t twi_wire_available(void);
    int twi_wire_read(void);
    int twi_wire_peek(void);
    void twi_wire_follower_tx_register_callback(void (*function)(void));
    void twi_wire_follower_rx_register_callback(void (*function)(int));
    
#ifdef	__cplusplus
}
#endif

#endif /* TWI_WIRE_H */
