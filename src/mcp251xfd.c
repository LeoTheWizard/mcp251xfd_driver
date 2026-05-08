/**
 * @file mcp251xfd.c
 * @brief Implementation of the MCP251xFD CAN controller device driver.
 *
 * @author Leo Walker
 *
 * @copyright Copyright (c) 2025 Leo Walker. Licensed under the MIT License.
 *            See LICENSE in the root directory for the full license text.
 *
 */

#include "stdlib.h"
#include "string.h"

#include "mcp251xfd.h"

/** @brief Buffer to store error messages */
#define ERROR_BUFFER_SIZE 128
static char error_buffer[ERROR_BUFFER_SIZE] = {0};

const char *mcp251xfd_get_error_msg(void)
{
    error_buffer[ERROR_BUFFER_SIZE - 1] = '\0'; // Ensure null termination
    return error_buffer;
}

/**
 * @brief Implementation of device instance.
 */
struct mcp2518fd_priv
{
    bool initialised; // Track if instance has been initialised for validation.

    void (*delay_func)(uint32_t microseconds);
    void (*ce_control_func)(void *iface, bool enable);
    void (*spi_transfer_func)(void *iface, const uint8_t *tx_data, uint8_t *rx_data, size_t length);

    mcp251xfd_config_t config;
    mcp251xfd_model_t model;
};

enum mcp2518fd_registers
{
    MCP2518FD_REG_C1CON = 0x0000,
    MCP2518FD_REG_C1NBTCFG = 0x0004,
    MCP2518FD_REG_C1DBTCFG = 0x0008,
    MCP2518FD_REG_C1TDC = 0x000C,
    MCP2518FD_REG_C1TBC = 0x0010,
    MCP2518FD_REG_C1TSCON = 0x0014,
    MCP2518FD_REG_C1VEC = 0x0018,
    MCP2518FD_REG_C1INT = 0x001C,
    MCP2518FD_REG_C1RXIF = 0x0020,
    MCP2518FD_REG_C1TXIF = 0x0024,
    MCP2518FD_REG_C1RXOVIF = 0x0028,
    MCP2518FD_REG_C1TXATIF = 0x002C,
    MCP2518FD_REG_C1TXREQ = 0x0030,
    MCP2518FD_REG_C1TREC = 0x0034,
    MCP2518FD_REG_C1BDIAG0 = 0x0038,
    MCP2518FD_REG_C1BDIAG1 = 0x003C,
    MCP2518FD_REG_C1TEFCON = 0x0040,
    MCP2518FD_REG_C1TEFSTA = 0x0044,
    MCP2518FD_REG_C1TEFUA = 0x0048,
    MCP2518FD_REG_C1TXQCON = 0x0050,
    MCP2518FD_REG_C1TXQSTA = 0x0054,
    MCP2518FD_REG_C1TXQUA = 0x0058,

    // FIFOs Start
    MCP2518FD_REG_C1FIFOCON1 = 0x005C,
    MCP2518FD_REG_C1FIFOSTA1 = 0x0060,
    MCP2518FD_REG_C1FIFOUA1 = 0x0064,

    //  Filter Control Start C1FLTCON0 - C1FLTCON7
    MCP2518FD_REG_C1FLTCON0 = 0x01D0,

    //  Filters & Masks Start
    MCP2518FD_REG_C1FLTOBJ0 = 0x01F0,
    MCP2518FD_REG_C1MASK0 = 0x01F4,

    MCP2518FD_REG_OSC = 0x0E00,     //  OSCILLATOR CONTROL REGISTER
    MCP2518FD_REG_IOCON = 0x0E04,   //  INPUT/OUTPUT CONTROL REGISTER
    MCP2518FD_REG_CRC = 0x0E08,     //  CRC REGISTER
    MCP2518FD_REG_ECCCON = 0x0E0C,  //  ECC CONTROL REGISTER
    MCP2518FD_REG_ECCSTAT = 0x0E10, //  ECC STATUS REGISTER
    MCP2518FD_REG_DEVID = 0x0E14    //  DEVICE ID REGISTER
};

#define MCP2518FD_REG_FIFOCON(fifo_number) (MCP2518FD_REG_C1FIFOCON1 + (fifo_number * 12))
#define MCP2518FD_REG_FIFOSTA(fifo_number) (MCP2518FD_REG_C1FIFOSTA1 + (fifo_number * 12))
#define MCP2518FD_REG_FIFOUA(fifo_number) (MCP2518FD_REG_C1FIFOUA1 + (fifo_number * 12))

#define MCP2518FD_REG_FLTCON(filter_number) (MCP2518FD_REG_C1FLTCON0 + (filter_number * 4))
#define MCP2518FD_REG_FLTOBJ(filter_number) (MCP2518FD_REG_C1FLTOBJ0 + (filter_number * 8))
#define MCP2518FD_REG_MASK0(mask_number) (MCP2518FD_REG_C1MASK0 + (mask_number * 8))

void mcp2518fd_reset(mcp2518fd_device_t *device)
{
}