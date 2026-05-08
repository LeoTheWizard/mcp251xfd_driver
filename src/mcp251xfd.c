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
#include "stdio.h"
#include "stdarg.h"

#include "mcp251xfd.h"

#pragma region Error Handling

/** @brief Buffer to store error messages */
#ifdef MCP251XFD_ENABLE_ERROR_MESSAGES
#define ERROR_BUFFER_SIZE 128
static char error_buffer[ERROR_BUFFER_SIZE] = {0};
#endif

const char *mcp251xfd_get_error_msg(void)
{
#ifdef MCP251XFD_ENABLE_ERROR_MESSAGES
    error_buffer[ERROR_BUFFER_SIZE - 1] = '\0'; // Ensure null termination
    return error_buffer;
#else
    return "";
#endif
}

/**
 * @brief If MCP251XFD_ENABLE_ERROR_MESSAGES is defined, formats an error message into the error buffer. Otherwise, does nothing.
 * @param format The format string (printf-style) for the error message.
 * @param ... Additional arguments for the format string.
 */
static inline void errorf(const char *format, ...)
{
#ifdef MCP251XFD_ENABLE_ERROR_MESSAGES
    va_list args;
    va_start(args, format);
    vsnprintf(error_buffer, ERROR_BUFFER_SIZE, format, args);
    va_end(args);
#endif
}

#pragma endregion Error Handling

#pragma region Definitions and Constants

enum mcp251xfd_registers
{
    MCP251XFD_REG_C1CON = 0x0000,
    MCP251XFD_REG_C1NBTCFG = 0x0004,
    MCP251XFD_REG_C1DBTCFG = 0x0008,
    MCP251XFD_REG_C1TDC = 0x000C,
    MCP251XFD_REG_C1TBC = 0x0010,
    MCP251XFD_REG_C1TSCON = 0x0014,
    MCP251XFD_REG_C1VEC = 0x0018,
    MCP251XFD_REG_C1INT = 0x001C,
    MCP251XFD_REG_C1RXIF = 0x0020,
    MCP251XFD_REG_C1TXIF = 0x0024,
    MCP251XFD_REG_C1RXOVIF = 0x0028,
    MCP251XFD_REG_C1TXATIF = 0x002C,
    MCP251XFD_REG_C1TXREQ = 0x0030,
    MCP251XFD_REG_C1TREC = 0x0034,
    MCP251XFD_REG_C1BDIAG0 = 0x0038,
    MCP251XFD_REG_C1BDIAG1 = 0x003C,
    MCP251XFD_REG_C1TEFCON = 0x0040,
    MCP251XFD_REG_C1TEFSTA = 0x0044,
    MCP251XFD_REG_C1TEFUA = 0x0048,
    MCP251XFD_REG_C1TXQCON = 0x0050,
    MCP251XFD_REG_C1TXQSTA = 0x0054,
    MCP251XFD_REG_C1TXQUA = 0x0058,

    // FIFOs Start
    MCP251XFD_REG_C1FIFOCON1 = 0x005C,
    MCP251XFD_REG_C1FIFOSTA1 = 0x0060,
    MCP251XFD_REG_C1FIFOUA1 = 0x0064,

    //  Filter Control Start C1FLTCON0 - C1FLTCON7
    MCP251XFD_REG_C1FLTCON0 = 0x01D0,

    //  Filters & Masks Start
    MCP251XFD_REG_C1FLTOBJ0 = 0x01F0,
    MCP251XFD_REG_C1MASK0 = 0x01F4,

    MCP251XFD_REG_OSC = 0x0E00,     //  OSCILLATOR CONTROL REGISTER
    MCP251XFD_REG_IOCON = 0x0E04,   //  INPUT/OUTPUT CONTROL REGISTER
    MCP251XFD_REG_CRC = 0x0E08,     //  CRC REGISTER
    MCP251XFD_REG_ECCCON = 0x0E0C,  //  ECC CONTROL REGISTER
    MCP251XFD_REG_ECCSTAT = 0x0E10, //  ECC STATUS REGISTER
    MCP251XFD_REG_DEVID = 0x0E14    //  DEVICE ID REGISTER
};

#define MCP251XFD_REG_FIFOCON(fifo_number) (MCP251XFD_REG_C1FIFOCON1 + (fifo_number * 12))
#define MCP251XFD_REG_FIFOSTA(fifo_number) (MCP251XFD_REG_C1FIFOSTA1 + (fifo_number * 12))
#define MCP251XFD_REG_FIFOUA(fifo_number) (MCP251XFD_REG_C1FIFOUA1 + (fifo_number * 12))

#define MCP251XFD_REG_FLTCON(filter_number) (MCP251XFD_REG_C1FLTCON0 + (filter_number * 4))
#define MCP251XFD_REG_FLTOBJ(filter_number) (MCP251XFD_REG_C1FLTOBJ0 + (filter_number * 8))
#define MCP251XFD_REG_MASK0(mask_number) (MCP251XFD_REG_C1MASK0 + (mask_number * 8))

#pragma endregion Definitions and Constants

#pragma region Instance Lifecycle

/**
 * @brief Implementation of device instance.
 */
struct mcp2518fd_priv
{
    bool initialised; // Track if instance has been initialised for validation.

    void (*delay)(uint32_t microseconds);
    void (*chip_enable)(void *iface, bool enable);
    void (*spi_transfer)(void *iface, const uint8_t *tx_data, uint8_t *rx_data, size_t length);

    mcp251xfd_config_t config;
    mcp251xfd_model_t model;
};

MCP251XFD *mcp251xfd_create_instance(void)
{
    MCP251XFD *instance = (MCP251XFD *)malloc(sizeof(MCP251XFD));
    if (instance == NULL)
    {
        errorf("Failed to allocate memory for MCP251xFD instance. malloc returned null.");
        return NULL;
    }
    memset(instance, 0, sizeof(MCP251XFD)); // Initialize all fields to null.
    return instance;
}

void mcp251xfd_destroy_instance(MCP251XFD *instance)
{
    if (instance == NULL)
        return; // Nothing to free.

    free(instance);
}

#pragma endregion Instance Lifecycle

#pragma region Device Control

mcp251xfd_return_t mcp251xfd_initialise(MCP251XFD *dev, mcp251xfd_config_t *config)
{
    /// Firstly validate the provided parameters.

    // Device is null
    if (dev == NULL)
    {
        errorf("MCP251xFD instance pointer is null. Provide some memory.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    // Configuration pointer is null
    if (config == NULL)
    {
        errorf("MCP251xFD configuration pointer is null. Provide a valid configuration.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    // Function pointers are null
    if (config->delay_func == NULL || config->chip_enable == NULL || config->spi_transfer == NULL)
    {
        errorf("MCP251xFD configuration function pointers cannot be null.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    return MCP251XFD_RETURN_OK;
}