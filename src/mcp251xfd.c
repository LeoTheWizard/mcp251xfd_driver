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

/**
 * @enum mcp251xfd_registers
 * @brief Register addresses for the MCP251xFD device.
 * These are used for SPI communication to read/write specific registers in the device.
 */
enum mcp251xfd_registers
{
    MCP251XFD_REG_C1CON = 0x0000, // CAN Control Register
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

/**
 * @enum mcp251xfd_cicon_bits
 * @brief Bit masks for the CAN Control register of the MCP251xFD device.
 */
enum mcp251xfd_cicon_bits
{
    MCP251XFD_C1CON_DNCNT_MASK = (0x1F << 0), // Device Net Filter Bit Number Mask.
    MCP251XFD_C1CON_ISOCRCEN = (0x01 << 5),   // Enable ISO CRC in CAN FD Frames bit.
    MCP251XFD_C1CON_PXEDIS = (0x01 << 6),     // Protocol Exception Event Detection Disabled bit
    MCP251XFD_C1CON_WAKFIL = (0x01 << 8),     // Enable CAN Bus Line Wake-up Filter bit.
    MCP251XFD_C1CON_WFT = (0x03 << 9),        // Selectable Wake-up Filter Time bits
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

#pragma region SPI Communication

/**
 * @enum mcp251xfd_spi_cmds
 * @brief SPI command codes for communicating with the MCP251xFD device.
 */
enum mcp251xfd_spi_cmds
{
    MCP251XFD_SPI_RESET = 0x00,
    MCP251XFD_SPI_READ = 0x03,
    MCP251XFD_SPI_WRITE = 0x02,
    MCP251XFD_SPI_READ_CRC = 0x0B,
    MCP251XFD_SPI_WRITE_CRC = 0x0A,
    MCP251XFD_SPI_WRITE_SAFE = 0x0C
};

/**
 * @brief Writes a data buffer to the MCP251xFD device memory provided a starting register address.
 * Address pointer automatically increments after each word written to memory.
 *
 * @param dev The MCP251xFD device instance.
 * @param reg_addr The starting register address to write to.
 * @param data The data buffer to be written to the device.
 * @param length The length of the data buffer in bytes.
 */
static void mcp251xfd_write_register(MCP251XFD *dev, uint16_t reg_addr, const uint8_t *data, size_t length)
{
    // Command buffer
    uint8_t cmd[2] = {MCP251XFD_SPI_WRITE << 4 | ((reg_addr >> 8) & 0x0F), reg_addr & 0xFF};

    dev->chip_enable(dev, true);
    dev->spi_transfer(dev, cmd, NULL, 2);       // Send write command and register address.
    dev->spi_transfer(dev, data, NULL, length); // Send data to be written.
    dev->chip_enable(dev, false);
}

/**
 * @brief Reads data from the MCP251xFD device memory starting from a specified register address into a provided buffer.
 * Address pointer automatically increments after each word read from memory.
 *
 * @param dev The MCP251xFD device instance.
 * @param reg_addr The starting register address to read from.
 * @param data The buffer to store the recieved data from the device.
 * @param length The length of the data to be read in bytes.
 */
static void mcp251xfd_read_register(MCP251XFD *dev, uint16_t reg_addr, uint8_t *data, size_t length)
{
    // Command buffer
    uint8_t cmd[2] = {MCP251XFD_SPI_READ << 4 | ((reg_addr >> 8) & 0x0F), reg_addr & 0xFF};

    dev->chip_enable(dev, true);
    dev->spi_transfer(dev, cmd, NULL, 2);       // Send read command and register address.
    dev->spi_transfer(dev, NULL, data, length); // Read data into buffer.
    dev->chip_enable(dev, false);
}

/**
 * @brief Writes a 32-bit word to the specified register address in the MCP251xFD device.
 */
static void mcp251xfd_write_word(MCP251XFD *dev, uint16_t reg_addr, uint32_t word)
{
    uint8_t data[4] = {
        (word >> 0) & 0xFF,
        (word >> 8) & 0xFF,
        (word >> 16) & 0xFF,
        (word >> 24) & 0xFF};
    mcp251xfd_write_register(dev, reg_addr, data, 4);
}

/**
 * @brief Reads a 32-bit word from the specified register address in the MCP251xFD device.
 * @return The 32-bit word read from the device.
 */
static uint32_t mcp251xfd_read_word(MCP251XFD *dev, uint16_t reg_addr)
{
    uint8_t data[4];
    mcp251xfd_read_register(dev, reg_addr, data, 4);
    return (data[0] << 0) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
}

/**
 * @brief Sends a reset command to the MCP251xFD device, which resets the device.
 * @note Device will enter configuration mode after returning from reset.
 *
 * @param dev The MCP251xFD device instance.
 */
static void mcp251xfd_reset_device(MCP251XFD *dev)
{
    uint8_t cmd[2] = {MCP251XFD_SPI_RESET << 4, 0};
    dev->chip_enable(dev, true);
    dev->spi_transfer(dev, cmd, NULL, 2); // Send reset command.
    dev->chip_enable(dev, false);
}

#pragma endregion SPI Communication

#pragma region Device Control

mcp251xfd_return_t mcp251xfd_request_opmode(MCP251XFD *dev, mcp251xfd_opmode_t mode)
{
    // Read current control register value.
    uint8_t c1con[4];
    mcp251xfd_read_register(dev, MCP251XFD_REG_C1CON, c1con, 4);

    // Clear current mode bits (bits 0-2).
    c1con[0] &= ~0x07;

    // Set new mode bits.
    c1con[0] |= (mode & 0x07);

    // Write updated control register value back to device.
    mcp251xfd_write_register(dev, MCP251XFD_REG_C1CON, c1con, 4);

    return MCP251XFD_RETURN_OK;
}

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