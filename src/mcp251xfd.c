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

#define TEST_DEVICE_PARAM(dev)                                                                                           \
    do                                                                                                                   \
    {                                                                                                                    \
        if (dev == NULL)                                                                                                 \
        {                                                                                                                \
            errorf("MCP251xFD instance pointer is null.");                                                               \
            return MCP251XFD_RETURN_INVALID_PARAM;                                                                       \
        }                                                                                                                \
        if (dev->initialised == false)                                                                                   \
        {                                                                                                                \
            errorf("MCP251xFD instance has not been initialised. Call mcp251xfd_initialise() before using the device."); \
            return MCP251XFD_RETURN_NOT_INITIALISED;                                                                     \
        }                                                                                                                \
    } while (0)

#define CHECK_DEV_PARAM(dev)                               \
    do                                                     \
    {                                                      \
        if (dev == NULL)                                   \
        {                                                  \
            errorf("MCP251xFD instance pointer is null."); \
            return MCP251XFD_RETURN_INVALID_PARAM;         \
        }                                                  \
    } while (0)

#pragma endregion Error Handling

#pragma region Definitions and Constants

/**
 * @enum mcp251xfd_registers
 * @brief Register addresses for the MCP251xFD device.
 * These are used for SPI communication to read/write specific registers in the device.
 */
enum mcp251xfd_registers
{
    MCP251XFD_REG_CICON = 0x0000, // CAN Control Register
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
    MCP251XFD_CICON_DNCNT_MASK = (0x1F << 0),  // Device Net Filter Bit Number Mask.
    MCP251XFD_CICON_ISOCRCEN = (0x01 << 5),    // Enable ISO CRC in CAN FD Frames bit.
    MCP251XFD_CICON_PXEDIS = (0x01 << 6),      // Protocol Exception Event Detection Disabled bit.
    MCP251XFD_CICON_WAKFIL = (0x01 << 8),      // Enable CAN Bus Line Wake-up Filter bit.
    MCP251XFD_CICON_WFT = (0x03 << 9),         // Selectable Wake-up Filter Time bits.
    MCP251XFD_CICON_BUSY = (0x01 << 11),       // CAN Module is Busy bit.
    MCP251XFD_CICON_BRSDIS = (0x01 << 12),     // Bit Rate Switching Disable bit.
    MCP251XFD_CICON_RTXAT = (0x01 << 16),      // Restrict Retransmission Attempts bit.
    MCP251XFD_CICON_ESIGM = (0x01 << 17),      // Transmit ESI in Gateway Mode bit.
    MCP251XFD_CICON_SERR2LOM = (0x01 << 18),   // Transition to Listen Only Mode on System Error bit.
    MCP251XFD_CICON_STEF = (0x01 << 19),       // Store in Transmit Event FIFO bit.
    MCP251XFD_CICON_TXQEN = (0x01 << 20),      // Transmit Queue Enable bit.
    MCP251XFD_CICON_OPMOD_MASK = (0x07 << 21), // Current operating mode status.
    MCP251XFD_CICON_REQOP_MASK = (0x07 << 24), // Requested operating mode bits.
    MCP251XFD_CICON_ABAT = (0x01 << 27),       // Abort All Pending Transmissions bit.
    MCP251XFD_CICON_TXBWS = (0x0F << 28)       // Transmit Bandwidth Sharing bits.
};
#define MCP251XFD_CICON_OPMOD_SFT 21
#define MCP251XFD_CICON_REQOP_SFT 24

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

    uint32_t (*time_us)(void);
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

    // Wait 100us for reset
    dev->delay(100);
}

#pragma endregion SPI Communication

#pragma region Device Control

mcp251xfd_return_t mcp251xfd_request_opmode(MCP251XFD *dev, mcp251xfd_opmode_t mode)
{
    CHECK_DEV_PARAM(dev);

    // Read current control register value.
    uint32_t cicon = mcp251xfd_read_word(dev, MCP251XFD_REG_CICON);

    // Clear current mode bits (bits 0-2).
    cicon &= ~MCP251XFD_CICON_REQOP_MASK;

    // Set new mode bits.
    cicon |= (mode << MCP251XFD_CICON_REQOP_SFT) & MCP251XFD_CICON_REQOP_MASK;

    // Write requested mode back to control register.
    mcp251xfd_write_word(dev, MCP251XFD_REG_CICON, cicon);

    return MCP251XFD_RETURN_OK;
}

mcp251xfd_return_t mcp251xfd_await_opmode(MCP251XFD *dev, mcp251xfd_opmode_t mode, uint32_t timeout_us)
{
    CHECK_DEV_PARAM(dev);

    uint32_t start_time = dev->time_us();

    while (true)
    {
        // Read current control register value.
        uint32_t cicon = mcp251xfd_read_word(dev, MCP251XFD_REG_CICON);

        // Check if current mode matches requested mode.
        if ((cicon & MCP251XFD_CICON_OPMOD_MASK) == ((mode << MCP251XFD_CICON_OPMOD_SFT) & MCP251XFD_CICON_OPMOD_MASK))
        {
            return MCP251XFD_RETURN_OK; // Desired mode achieved.
        }

        // Check for timeout condition here using start_time and current time.
        if (dev->time_us() - start_time > timeout_us)
        {
            return MCP251XFD_RETURN_TIMEOUT; // Timeout occurred.
        }

        dev->delay(10); // Delay briefly before checking again to avoid busy-waiting.
    }
}

mcp251xfd_return_t mcp251xfd_change_opmode(MCP251XFD *dev, mcp251xfd_opmode_t mode, uint32_t timeout_us)
{
    CHECK_DEV_PARAM(dev);

    mcp251xfd_return_t result = mcp251xfd_request_opmode(dev, mode);
    if (result != MCP251XFD_RETURN_OK)
    {
        return result; // Return error from request_opmode if it failed.
    }

    return mcp251xfd_await_opmode(dev, mode, timeout_us); // Wait for mode change to take effect and return result.
}

mcp251xfd_return_t mcp251xfd_get_opmode(MCP251XFD *dev, mcp251xfd_opmode_t *mode)
{
    CHECK_DEV_PARAM(dev);

    if (mode == NULL)
    {
        errorf("Output parameter for current mode cannot be null.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    // Read current control register value.
    uint32_t cicon = mcp251xfd_read_word(dev, MCP251XFD_REG_CICON);

    // Extract current mode bits and store in output parameter.
    *mode = (mcp251xfd_opmode_t)((cicon & MCP251XFD_CICON_OPMOD_MASK) >> 21);

    return MCP251XFD_RETURN_OK;
}

mcp251xfd_return_t mcp251xfd_initialise(MCP251XFD *dev, mcp251xfd_config_t *config)
{
    /// Firstly validate the provided parameters.

    // Device is null
    CHECK_DEV_PARAM(dev);

    // Configuration pointer is null
    if (config == NULL)
    {
        errorf("MCP251xFD configuration pointer is null. Provide a valid configuration.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    // Function pointers are null
    if (config->elapsed_us == NULL || config->delay_func == NULL || config->chip_enable == NULL || config->spi_transfer == NULL)
    {
        errorf("MCP251xFD configuration function pointers cannot be null.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    /// Validate configuration parameters.
    return MCP251XFD_RETURN_OK;
}