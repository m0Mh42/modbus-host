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
#include "app_config.h"
#include "register_map.h"
#include "hal_uart.h"
#include "hal_time.h"

#include "nanomodbus.h"

static nmbs_t nmbs;
static uint8_t s_configured_slave_address = MODBUS_SLAVE_ADDRESS;

/* ------------------------------------------------------------------ */
/* Transport: byte-stream read/write/flush nanoMODBUS calls into.     */
/* ------------------------------------------------------------------ */

/* Read up to `count` bytes, waiting at most byte_timeout_ms total.
 * Returns the number of bytes actually read (>=0). */
static int32_t uart_read(uint8_t *buf, uint16_t count, int32_t byte_timeout_ms, void *arg)
{
    (void)arg;
    uint16_t n = 0;
    uint32_t start = hal_millis();

    while (n < count)
    {
        if (hal_uart_read_byte(&buf[n]))
        {
            n++;
            start = hal_millis(); /* Reset timer on each received byte */
        }
        else if (byte_timeout_ms == 0)
        {
            break; /* Non-blocking: return immediately when empty */
        }
        else if (byte_timeout_ms > 0 &&
                 (int32_t)(hal_millis() - start) > byte_timeout_ms)
        {
            break; /* Timed out waiting for more bytes */
        }
    }
    return n;
}

/* Write `count` bytes. RS-485 driver handles turnaround delay and TC synchronization. */
static int32_t uart_write(const uint8_t *buf, uint16_t count, int32_t byte_timeout_ms, void *arg)
{
    (void)byte_timeout_ms;
    (void)arg;
    hal_uart_set_tx_enable(1);
    hal_uart_write(buf, count);
    hal_uart_set_tx_enable(0);
    return count;
}

/* Flush the UART RX line on error recovery. */
static void uart_flush(nmbs_t *handle, void *arg)
{
    (void)handle;
    (void)arg;
    hal_uart_flush_rx();
}

/* ------------------------------------------------------------------ */
/* Data-model callbacks: bridge nanoMODBUS <-> register_map.          */
/* ------------------------------------------------------------------ */

static nmbs_error cb_read_coils(uint16_t address, uint16_t quantity,
                                nmbs_bitfield coils_out, uint8_t unit_id, void *arg)
{
    (void)unit_id;
    (void)arg;
    for (uint16_t i = 0; i < quantity; i++)
    {
        bool v = false;
        if (!register_map_read_coil(address + i, &v))
        {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
        nmbs_bitfield_write(coils_out, i, v ? 1 : 0);
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_write_coils(uint16_t address, uint16_t quantity,
                                 const nmbs_bitfield coils, uint8_t unit_id, void *arg)
{
    (void)unit_id;
    (void)arg;
    for (uint16_t i = 0; i < quantity; i++)
    {
        bool v = nmbs_bitfield_read(coils, i) ? true : false;
        if (!register_map_write_coil(address + i, v))
        {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_write_single_coil(uint16_t address, bool value, uint8_t unit_id, void *arg)
{
    (void)unit_id;
    (void)arg;
    if (!register_map_write_coil(address, value))
    {
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_read_discrete_inputs(uint16_t address, uint16_t quantity,
                                          nmbs_bitfield inputs_out, uint8_t unit_id, void *arg)
{
    (void)unit_id;
    (void)arg;
    for (uint16_t i = 0; i < quantity; i++)
    {
        bool v = false;
        if (!register_map_read_discrete_input(address + i, &v))
        {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
        nmbs_bitfield_write(inputs_out, i, v ? 1 : 0);
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_read_holding(uint16_t address, uint16_t quantity,
                                  uint16_t *registers_out, uint8_t unit_id, void *arg)
{
    (void)unit_id;
    (void)arg;
    for (uint16_t i = 0; i < quantity; i++)
    {
        if (!register_map_read_holding(address + i, &registers_out[i]))
        {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_write_holding(uint16_t address, uint16_t quantity,
                                   const uint16_t *registers, uint8_t unit_id, void *arg)
{
    (void)unit_id;
    (void)arg;
    for (uint16_t i = 0; i < quantity; i++)
    {
        if (!register_map_write_holding(address + i, registers[i]))
        {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_write_single_register(uint16_t address, uint16_t value, uint8_t unit_id, void *arg)
{
    (void)unit_id;
    (void)arg;
    if (!register_map_write_holding(address, value))
    {
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_read_input(uint16_t address, uint16_t quantity,
                                uint16_t *registers_out, uint8_t unit_id, void *arg)
{
    (void)unit_id;
    (void)arg;
    for (uint16_t i = 0; i < quantity; i++)
    {
        if (!register_map_read_input(address + i, &registers_out[i]))
        {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
    }
    return NMBS_ERROR_NONE;
}

/* ------------------------------------------------------------------ */

void modbus_init(uint8_t slave_address, uint32_t baudrate)
{
    s_configured_slave_address = slave_address;

    hal_uart_init(baudrate);

    nmbs_platform_conf platform;
    nmbs_platform_conf_create(&platform);
    platform.transport = NMBS_TRANSPORT_RTU;
    platform.read = uart_read;
    platform.write = uart_write;
    platform.flush = uart_flush;
    platform.arg = NULL;

    nmbs_callbacks callbacks;
    nmbs_callbacks_create(&callbacks);
    callbacks.read_coils = cb_read_coils;
    callbacks.write_multiple_coils = cb_write_coils;
    callbacks.write_single_coil = cb_write_single_coil;
    callbacks.read_discrete_inputs = cb_read_discrete_inputs;
    callbacks.read_holding_registers = cb_read_holding;
    callbacks.write_multiple_registers = cb_write_holding;
    callbacks.write_single_register = cb_write_single_register;
    callbacks.read_input_registers = cb_read_input;

    if (nmbs_server_create(&nmbs, slave_address, &platform, &callbacks) != NMBS_ERROR_NONE)
    {
        return;
    }

    /* Set non-blocking initial read timeout (0ms) so superloop never hangs on idle bus.
     * Use generous 20ms for intra-frame byte timeout. */
    nmbs_set_read_timeout(&nmbs, 0);
    nmbs_set_byte_timeout(&nmbs, 20);
}

void modbus_task(void)
{
    /* Only poll when a complete Modbus RTU frame has been received and confirmed
     * by the T3.5 silent interval. This guarantees:
     * 1. All bytes of the request are already in the ring buffer (zero wait).
     * 2. Yield loops during CS1237 wait-edges never prematurely read partial frames.
     * 3. nanoMODBUS will never timeout mid-frame and flush the buffer.
     */
    if (hal_uart_is_frame_ready())
    {
        uint8_t unit_id = 0;
        if (hal_uart_peek_byte(0, &unit_id))
        {
            /* Strictly accept only the configured slave address or broadcast (0) */
            if (unit_id == s_configured_slave_address || unit_id == 0)
            {
                nmbs_server_poll(&nmbs);
            }
        }
        /* Discard processed or non-matching frames so receiver is ready for next frame */
        hal_uart_flush_rx();
    }
}

void modbus_set_slave_address(uint8_t slave_address)
{
    s_configured_slave_address = slave_address;
    nmbs.address_rtu = slave_address;
}

uint8_t modbus_get_slave_address(void)
{
    return s_configured_slave_address;
}
