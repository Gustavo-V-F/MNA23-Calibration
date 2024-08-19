#include <stdbool.h>
#include "twi.h"
#include "twi_wire.h"

static uint8_t twi_wire_rx_buffer[TWI_WIRE_BUFFER_LENGTH];
static uint8_t twi_wire_rx_buffer_idx = 0;
static uint8_t twi_wire_rx_buffer_length = 0;

static uint8_t twi_wire_tx_address = 0;
static uint8_t twi_wire_tx_buffer[TWI_WIRE_BUFFER_LENGTH];
static uint8_t twi_wire_tx_buffer_idx = 0;
static uint8_t twi_wire_tx_buffer_length = 0;

static uint8_t twi_wire_transmitting = 0;
static void (*twi_wire_follower_tx_callback)(void);
static void (*twi_wire_follower_rx_callback)(int);

static void twi_wire_follower_tx_event(void) {
    if (!twi_wire_follower_tx_callback) {
        return;
    }

    twi_wire_tx_buffer_idx = 0;
    twi_wire_tx_buffer_length = 0;

    twi_wire_follower_tx_callback();
}

static void twi_wire_follower_rx_event(uint8_t* data, int data_size) {
    uint8_t i = 0;

    if (!twi_wire_follower_rx_callback) {
        return;
    }

    if (twi_wire_rx_buffer_idx < twi_wire_rx_buffer_length) {
        return;
    }

    for (i = 0; i < data_size; i++) {
        twi_wire_rx_buffer[i] = data[i];
    }

    twi_wire_rx_buffer_idx = 0;
    twi_wire_rx_buffer_length = data_size;

    twi_wire_follower_rx_callback(data_size);
}

static void twi_wire_init(bool pullup) {
    twi_wire_rx_buffer_idx = 0;
    twi_wire_rx_buffer_length = 0;

    twi_wire_tx_buffer_idx = 0;
    twi_wire_tx_buffer_length = 0;

    twi_init(pullup);
    twi_wire_timeout(3000, true);
    twi_attachSlaveTxEvent(twi_wire_follower_tx_event);
    twi_attachSlaveRxEvent(twi_wire_follower_rx_event);
}

void twi_wire_leader_init(bool pullup) {
    twi_wire_init(pullup);
}

void twi_wire_follower_init(uint8_t address, bool pullup) {
    twi_wire_init(pullup);
    twi_setAddress(address);
}

void twi_wire_disable(void) {
    twi_disable();
}

void twi_wire_frequency(uint32_t freq) {
    twi_setFrequency(freq);
}

bool twi_wire_get_timeout_flag(void) {
    return (twi_manageTimeoutFlag(false));
}

void twi_wire_clear_timeout_flag(void) {
    twi_manageTimeoutFlag(true);
}

void twi_wire_timeout(uint32_t timeout, bool reset_with_timeout) {
    twi_setTimeoutInMicros(timeout, reset_with_timeout);
}

uint8_t twi_wire_request_with_reg_from(
        uint8_t address,
        uint8_t size,
        uint32_t internal_address,
        uint8_t internal_size,
        uint8_t send_stop) {
    uint8_t read = 0;
    
    if (internal_size > 0) {
        twi_wire_begin_transmission(address);
        
        if (internal_size > 3) {
            internal_size = 3;
        }

        while(internal_size-- > 0){
            twi_wire_write_byte((uint8_t)(internal_address >> (internal_size*8)));
            twi_wire_end_transmission(false);
        }
    }
    
    if (size > TWI_WIRE_BUFFER_LENGTH) {
        size = TWI_WIRE_BUFFER_LENGTH;
    }
    
    read = twi_readFrom(address, twi_wire_rx_buffer, size, send_stop);
    
    twi_wire_rx_buffer_idx = 0;
    twi_wire_rx_buffer_length = read;
    
    return read;
}

uint8_t twi_wire_request_from(uint8_t address, uint8_t size, uint8_t send_stop){
    return twi_wire_request_with_reg_from(address, size, 0, 0, send_stop);
}

void twi_wire_begin_transmission(uint8_t address) {
    twi_wire_transmitting = 1;

    twi_wire_tx_address = address;

    twi_wire_tx_buffer_idx = 0;
    twi_wire_tx_buffer_length = 0;
}

uint8_t twi_wire_end_transmission(uint8_t send_stop) {
    uint8_t ret = twi_writeTo(
            twi_wire_tx_address,
            twi_wire_tx_buffer,
            twi_wire_tx_buffer_length,
            1,
            send_stop);

    twi_wire_tx_buffer_idx = 0;
    twi_wire_tx_buffer_length = 0;
    twi_wire_transmitting = 0;
    return ret;
}

size_t twi_wire_write_byte(uint8_t data){
    if (twi_wire_transmitting) {
        if (twi_wire_tx_buffer_length >= TWI_WIRE_BUFFER_LENGTH) {
            return 0;
        }
        
        twi_wire_tx_buffer[twi_wire_tx_buffer_idx++] = data;
        twi_wire_tx_buffer_length = twi_wire_tx_buffer_idx;
    } else {
        twi_transmit(&data, 1);
    }
    return 1;
}

size_t twi_wire_write(const uint8_t *data, size_t size) {
    uint8_t i = 0;
    
    if (twi_wire_transmitting) {
        
        for (i = 0; i < size; i++) {
            twi_wire_write_byte(data[i]);
        }
    } else {
        twi_transmit(data, size);
    }
    return size;
}

uint8_t twi_wire_available(void){
    return twi_wire_rx_buffer_length - twi_wire_rx_buffer_idx;
}

int twi_wire_read(void){
    int value = -1;
    
    if (twi_wire_rx_buffer_idx < twi_wire_rx_buffer_length) {
        value = twi_wire_rx_buffer[twi_wire_rx_buffer_idx++];
    }
    
    return value;
}

int twi_wire_peek(void){
    int value = -1;
    
    if (twi_wire_rx_buffer_idx < twi_wire_rx_buffer_length) {
        value = twi_wire_rx_buffer[twi_wire_rx_buffer_idx];
    }
    
    return value;
}

void twi_wire_follower_tx_register_callback(void (*function)(void)) {
    twi_wire_follower_tx_callback = function;
}

void twi_wire_follower_rx_register_callback(void (*function)(int)) {
    twi_wire_follower_rx_callback = function;
}