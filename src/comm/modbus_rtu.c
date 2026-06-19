/**
 * @file modbus_rtu.c
 * @brief Modbus RTU slave implementation on top of nanoMODBUS.
 *
 * Wiring:
 *   transport  : nanoMODBUS read/write fn -> hal_uart (RS-485 DE handled here)
 *   data model : nanoMODBUS callbacks     -> register_map
 *
 * Supported function codes (provided by nanoMODBUS):
 *   FC01 read coils, FC05/FC15 write coils       -> relays
 *   FC03 read holding, FC06/FC16 write holding   -> relay mask, PWM duty
 *   FC04 read input registers                    -> sensors
 *
 * If you change which callbacks are populated, unsupported requests simply get
 * an ILLEGAL FUNCTION exception from nanoMODBUS.
 */
#include "modbus_rtu.h"
#include "register_map.h"
#include "hal_uart.h"
#include "hal_time.h"

#include "nanomodbus.h"

static nmbs_t nmbs;

/* ------------------------------------------------------------------ */
/* Transport: byte-stream read/write the library calls into.          */
/* ------------------------------------------------------------------ */

/* Read up to `count` bytes, waiting at most byte_timeout_ms total.
 * Returns the number of bytes actually read (>=0). */
static int32_t uart_read(uint8_t *buf, uint16_t count, int32_t byte_timeout_ms, void *arg)
{
    (void)arg;
    uint16_t n = 0;
    uint32_t start = hal_millis();
    while (n < count) {
        if (hal_uart_read_byte(&buf[n])) {
            n++;
        } else if (byte_timeout_ms >= 0 &&
                   (int32_t)(hal_millis() - start) > byte_timeout_ms) {
            break;     /* timed out waiting for more bytes */
        }
    }
    return n;
}

/* Write `count` bytes. RS-485 driver is enabled for the duration of the frame
 * (hal_uart_write blocks until the last byte has fully shifted out). */
static int32_t uart_write(const uint8_t *buf, uint16_t count, int32_t byte_timeout_ms, void *arg)
{
    (void)byte_timeout_ms; (void)arg;
    hal_uart_set_tx_enable(1);
    hal_uart_write(buf, count);
    hal_uart_set_tx_enable(0);
    return count;
}

/* ------------------------------------------------------------------ */
/* Data-model callbacks: bridge nanoMODBUS <-> register_map.          */
/* ------------------------------------------------------------------ */

static nmbs_error cb_read_coils(uint16_t address, uint16_t quantity,
                                nmbs_bitfield coils_out, uint8_t unit_id, void *arg)
{
    (void)unit_id; (void)arg;
    for (uint16_t i = 0; i < quantity; i++) {
        bool v = false;
        if (!register_map_read_coil(address + i, &v)) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
        nmbs_bitfield_write(coils_out, i, v ? 1 : 0);
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_write_coils(uint16_t address, uint16_t quantity,
                                 const nmbs_bitfield coils, uint8_t unit_id, void *arg)
{
    (void)unit_id; (void)arg;
    for (uint16_t i = 0; i < quantity; i++) {
        bool v = nmbs_bitfield_read(coils, i) ? true : false;
        if (!register_map_write_coil(address + i, v)) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_read_holding(uint16_t address, uint16_t quantity,
                                  uint16_t *registers_out, uint8_t unit_id, void *arg)
{
    (void)unit_id; (void)arg;
    for (uint16_t i = 0; i < quantity; i++) {
        if (!register_map_read_holding(address + i, &registers_out[i])) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_write_holding(uint16_t address, uint16_t quantity,
                                   const uint16_t *registers, uint8_t unit_id, void *arg)
{
    (void)unit_id; (void)arg;
    for (uint16_t i = 0; i < quantity; i++) {
        if (!register_map_write_holding(address + i, registers[i])) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_read_input(uint16_t address, uint16_t quantity,
                                uint16_t *registers_out, uint8_t unit_id, void *arg)
{
    (void)unit_id; (void)arg;
    for (uint16_t i = 0; i < quantity; i++) {
        if (!register_map_read_input(address + i, &registers_out[i])) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
    }
    return NMBS_ERROR_NONE;
}

/* ------------------------------------------------------------------ */

void modbus_init(uint8_t slave_address, uint32_t baudrate)
{
    hal_uart_init(baudrate);

    nmbs_platform_conf platform = {0};
    platform.transport = NMBS_TRANSPORT_RTU;
    platform.read  = uart_read;
    platform.write = uart_write;
    platform.arg   = NULL;

    nmbs_callbacks callbacks = {0};
    callbacks.read_coils               = cb_read_coils;
    callbacks.write_multiple_coils     = cb_write_coils;
    callbacks.read_holding_registers   = cb_read_holding;
    callbacks.write_multiple_registers = cb_write_holding;
    callbacks.read_input_registers     = cb_read_input;

    if (nmbs_server_create(&nmbs, slave_address, &platform, &callbacks) != NMBS_ERROR_NONE) {
        /* TODO: signal a fatal init error (LED, etc.). */
        return;
    }

    /* Inter-frame / byte timeouts. RTU wants ~3.5 char silence between frames;
     * keep these modest so the poll stays responsive. Tune for your baud. */
    nmbs_set_read_timeout(&nmbs, 50);
    nmbs_set_byte_timeout(&nmbs, 10);
}

void modbus_task(void)
{
    /* Processes at most one request per call; returns NMBS_ERROR_TIMEOUT when
     * the bus is idle, which is normal and ignored here. */
    nmbs_server_poll(&nmbs);
}
