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

#ifdef MCP251XFD_CHECK_NULL_PARAM
#define CHECK_NULL_PARAM(param)                    \
    do                                             \
    {                                              \
        if (param == NULL)                         \
        {                                          \
            errorf("Null parameter: " #param);     \
            return MCP251XFD_RETURN_INVALID_PARAM; \
        }                                          \
    } while (0)
#else
#define CHECK_NULL_PARAM(param) ((void)0) // No-op if null parameter checking is disabled.
#endif

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
    MCP251XFD_REG_CINBTCFG = 0x0004,
    MCP251XFD_REG_CIDBTCFG = 0x0008,
    MCP251XFD_REG_CITDC = 0x000C,
    MCP251XFD_REG_CITBC = 0x0010,
    MCP251XFD_REG_CITSCON = 0x0014,
    MCP251XFD_REG_CIVEC = 0x0018,
    MCP251XFD_REG_CIINT = 0x001C,
    MCP251XFD_REG_CIRXIF = 0x0020,
    MCP251XFD_REG_CITXIF = 0x0024,
    MCP251XFD_REG_CIRXOVIF = 0x0028,
    MCP251XFD_REG_CITXATIF = 0x002C,
    MCP251XFD_REG_CITXREQ = 0x0030,
    MCP251XFD_REG_CITREC = 0x0034,
    MCP251XFD_REG_CIBDIAG0 = 0x0038,
    MCP251XFD_REG_CIBDIAG1 = 0x003C,
    MCP251XFD_REG_CITEFCON = 0x0040,
    MCP251XFD_REG_CITEFSTA = 0x0044,
    MCP251XFD_REG_CITEFUA = 0x0048,
    MCP251XFD_REG_CITXQCON = 0x0050,
    MCP251XFD_REG_CITXQSTA = 0x0054,
    MCP251XFD_REG_CITXQUA = 0x0058,

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

#define MCP251XFD_RAM_START 0x0400
#define MCP251XFD_RAM_END 0x04FF

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

enum mcp251xfd_osc_bits
{
    MCP251XFD_OSC_PLLEN = (0x01 << 0),    // PLL Enable: 1 = 10x PLL, 0 = direct oscillator
    MCP251XFD_OSC_OSCDIS = (0x01 << 2),   // Oscillator Disabled (MCP2518FD only): 1 = ext clock on CLKI, 0 = crystal circuit enabled
    MCP251XFD_OSC_SCLKDIV = (0x01 << 4),  // System Clock Divisor: 0 = /1, 1 = /2
    MCP251XFD_OSC_CLKODIV = (0x03 << 5),  // CLKO Pin Divisor: 00=/1, 01=/2, 10=/4, 11=/10
    MCP251XFD_OSC_PLLRDY = (0x01 << 8),   // PLL Ready (read-only)
    MCP251XFD_OSC_OSCRDY = (0x01 << 10),  // Oscillator Ready (read-only)
    MCP251XFD_OSC_SCLKRDY = (0x01 << 12), // System Clock Ready (read-only)
};
#define MCP251XFD_OSC_CLKODIV_SFT 5

enum mcp251xfd_ecccon_bits
{
    MCP251XFD_ECCCON_ECCEN = (0x01 << 0),  // ECC Enable.
    MCP251XFD_ECCCON_SECIE = (0x01 << 1),  // Single Error Correction Interrupt Enable.
    MCP251XFD_ECCCON_DEDIE = (0x01 << 2),  // Double Error Detection Interrupt Enable.
    MCP251XFD_ECCCON_PARITY = (0x3F << 8), // Parity bits for ECC syndrome injection.
};
#define MCP251XFD_ECCCON_PARITY_SFT 8

enum mcp251xfd_eccstat_bits
{
    MCP251XFD_ECCSTAT_SECIF = (0x01 << 1),   // Single Error Correction Interrupt Flag.
    MCP251XFD_ECCSTAT_DEDIF = (0x01 << 2),   // Double Error Detection Interrupt Flag.
    MCP251XFD_ECCSTAT_ERRADDR = (0xFF << 8), // Low byte of RAM address where last error was detected.
};
#define MCP251XFD_ECCSTAT_ERRADDR_SFT 8

enum mcp251xfd_fifocon_bits
{
    MCP251XFD_FIFOCON_FSIZE_MASK = (0x1F << 0),   // FIFO depth (0=1 obj, 31=32 obj).
    MCP251XFD_FIFOCON_TXEN = (0x01 << 7),         // 1 = TX FIFO, 0 = RX FIFO.
    MCP251XFD_FIFOCON_RXOVIE = (0x01 << 8),       // RX overflow interrupt enable.
    MCP251XFD_FIFOCON_TXATIE = (0x01 << 9),       // TX attempt interrupt enable.
    MCP251XFD_FIFOCON_TFERFFIE = (0x01 << 10),    // Empty/full interrupt enable.
    MCP251XFD_FIFOCON_TFHRFHIE = (0x01 << 11),    // Half empty/full interrupt enable.
    MCP251XFD_FIFOCON_TFNRFNIE = (0x01 << 12),    // Not full/not empty interrupt enable.
    MCP251XFD_FIFOCON_FRESET = (0x01 << 13),      // Reset FIFO head/tail pointers.
    MCP251XFD_FIFOCON_TXPRI_MASK = (0x1F << 16),  // TX message priority (TX only).
    MCP251XFD_FIFOCON_RTREN = (0x01 << 22),       // Auto-RTR enable (TX only).
    MCP251XFD_FIFOCON_PLSIZE_MASK = (0x07 << 24), // Payload size.
};
#define MCP251XFD_FIFOCON_FSIZE_SFT 0
#define MCP251XFD_FIFOCON_TXPRI_SFT 16
#define MCP251XFD_FIFOCON_PLSIZE_SFT 24

#define MCP251XFD_REG_FIFOCON(fifo_number) (MCP251XFD_REG_C1FIFOCON1 + (fifo_number * 12))
#define MCP251XFD_REG_FIFOSTA(fifo_number) (MCP251XFD_REG_C1FIFOSTA1 + (fifo_number * 12))
#define MCP251XFD_REG_FIFOUA(fifo_number) (MCP251XFD_REG_C1FIFOUA1 + (fifo_number * 12))

#define MCP251XFD_REG_FLTCON(filter_number) (MCP251XFD_REG_C1FLTCON0 + (filter_number * 4))
#define MCP251XFD_REG_FLTOBJ(filter_number) (MCP251XFD_REG_C1FLTOBJ0 + (filter_number * 8))
#define MCP251XFD_REG_MASK0(mask_number) (MCP251XFD_REG_C1MASK0 + (mask_number * 8))

// CRC-16 lookup table, polynomial 0x8005 (x^16+x^15+x^2+1), MSB-first, no reflection.
// Used for SPI READ_CRC / WRITE_CRC commands. Update: crc = (crc<<8) ^ crc16_table[(crc>>8) ^ byte].
// clang-format off
static const uint16_t crc16_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202,
};
// clang-format on

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
    mcp251xfd_fosc_t sysclk;
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

static uint16_t crc16_compute(uint16_t crc, const uint8_t *data, size_t len)
{
    while (len--)
        crc = (crc << 8) ^ crc16_table[(crc >> 8) ^ *data++];
    return crc;
}

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
 * @brief Writes a data buffer to the MCP251xFD device with CRC protection.
 * Appends a CRC-16 computed over the command header and data to the SPI frame.
 */
static void mcp251xfd_write_register_crc(MCP251XFD *dev, uint16_t reg_addr, const uint8_t *data, size_t length)
{
    uint8_t cmd[3] = {
        MCP251XFD_SPI_WRITE_CRC << 4 | ((reg_addr >> 8) & 0x0F),
        reg_addr & 0xFF,
        (uint8_t)(length / 4)};

    uint16_t crc = crc16_compute(0xFFFF, cmd, 3);
    crc = crc16_compute(crc, data, length);
    uint8_t crc_bytes[2] = {(uint8_t)(crc >> 8), (uint8_t)(crc & 0xFF)};

    dev->chip_enable(dev, true);
    dev->spi_transfer(dev, cmd, NULL, 3);
    dev->spi_transfer(dev, data, NULL, length);
    dev->spi_transfer(dev, crc_bytes, NULL, 2);
    dev->chip_enable(dev, false);
}

/**
 * @brief Reads data from the MCP251xFD device with CRC verification.
 * Sends a CRC of the command header; verifies the CRC returned by the device over the full frame.
 *
 * @return MCP251XFD_RETURN_OK on success, MCP251XFD_RETURN_CRC_ERROR if the received CRC does not match.
 */
static mcp251xfd_return_t mcp251xfd_read_register_crc(MCP251XFD *dev, uint16_t reg_addr, uint8_t *data, size_t length)
{
    uint8_t cmd[3] = {
        MCP251XFD_SPI_READ_CRC << 4 | ((reg_addr >> 8) & 0x0F),
        reg_addr & 0xFF,
        (uint8_t)(length / 4)};

    uint16_t cmd_crc = crc16_compute(0xFFFF, cmd, 3);
    uint8_t cmd_crc_bytes[2] = {(uint8_t)(cmd_crc >> 8), (uint8_t)(cmd_crc & 0xFF)};
    uint8_t rx_crc_bytes[2];

    dev->chip_enable(dev, true);
    dev->spi_transfer(dev, cmd, NULL, 3);
    dev->spi_transfer(dev, cmd_crc_bytes, NULL, 2);
    dev->spi_transfer(dev, NULL, data, length);
    dev->spi_transfer(dev, NULL, rx_crc_bytes, 2);
    dev->chip_enable(dev, false);

    // Device CRC covers cmd header + received data.
    uint16_t expected = crc16_compute(0xFFFF, cmd, 3);
    expected = crc16_compute(expected, data, length);
    uint16_t received = ((uint16_t)rx_crc_bytes[0] << 8) | rx_crc_bytes[1];

    return (received == expected) ? MCP251XFD_RETURN_OK : MCP251XFD_RETURN_CRC_ERROR;
}

/**
 * @brief Writes a 32-bit word to the specified register address with CRC protection.
 */
static void mcp251xfd_write_word_crc(MCP251XFD *dev, uint16_t reg_addr, uint32_t word)
{
    uint8_t data[4] = {
        (word >> 0) & 0xFF,
        (word >> 8) & 0xFF,
        (word >> 16) & 0xFF,
        (word >> 24) & 0xFF};
    mcp251xfd_write_register_crc(dev, reg_addr, data, 4);
}

/**
 * @brief Reads a 32-bit word from the specified register address with CRC verification.
 *
 * @param word Output pointer for the read value; only written on MCP251XFD_RETURN_OK.
 * @return MCP251XFD_RETURN_OK on success, MCP251XFD_RETURN_CRC_ERROR on CRC mismatch.
 */
static mcp251xfd_return_t mcp251xfd_read_word_crc(MCP251XFD *dev, uint16_t reg_addr, uint32_t *word)
{
    uint8_t data[4];
    mcp251xfd_return_t ret = mcp251xfd_read_register_crc(dev, reg_addr, data, 4);
    if (ret == MCP251XFD_RETURN_OK)
        *word = ((uint32_t)data[0] << 0) | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    return ret;
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
    CHECK_NULL_PARAM(dev);

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
    CHECK_NULL_PARAM(dev);

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
    CHECK_NULL_PARAM(dev);

    mcp251xfd_return_t result = mcp251xfd_request_opmode(dev, mode);
    if (result != MCP251XFD_RETURN_OK)
    {
        return result; // Return error from request_opmode if it failed.
    }

    return mcp251xfd_await_opmode(dev, mode, timeout_us); // Wait for mode change to take effect and return result.
}

mcp251xfd_return_t mcp251xfd_get_opmode(MCP251XFD *dev, mcp251xfd_opmode_t *mode)
{
    CHECK_NULL_PARAM(dev);
    CHECK_NULL_PARAM(mode);

    // Read current control register value.
    uint32_t cicon = mcp251xfd_read_word(dev, MCP251XFD_REG_CICON);

    // Extract current mode bits and store in output parameter.
    *mode = (mcp251xfd_opmode_t)((cicon & MCP251XFD_CICON_OPMOD_MASK) >> 21);

    return MCP251XFD_RETURN_OK;
}

/**
 * @brief Configures the oscillator of the MCP251xFD device based on the selected external clock frequency in the configuration.
 *
 * @param dev The MCP251xFD device instance.
 * @param fosc The selected external clock frequency from the configuration.
 *
 * @return mcp251xfd_return_t indicating the result of the operation, including timeout if the oscillator fails to stabilize within the expected time.
 */
static mcp251xfd_return_t mcp251xfd_configure_osc(MCP251XFD *dev, mcp251xfd_fosc_t fosc)
{
    uint32_t osc = 0;
    uint32_t ready_mask = MCP251XFD_OSC_OSCRDY | MCP251XFD_OSC_SCLKRDY;

    // If using 4Mhz oscillator enable 10x pll to achieve 40Mhz system clock.
    if (fosc == MCP251XFD_FOSC_4MHZ)
    {
        osc = MCP251XFD_OSC_PLLEN;
        ready_mask = MCP251XFD_OSC_PLLRDY | MCP251XFD_OSC_SCLKRDY;
    }

    mcp251xfd_write_word(dev, MCP251XFD_REG_OSC, osc);

    uint32_t start = dev->time_us();
    while ((mcp251xfd_read_word(dev, MCP251XFD_REG_OSC) & ready_mask) != ready_mask)
    {
        if (dev->time_us() - start > 10000)
            return MCP251XFD_RETURN_TIMEOUT;
        dev->delay(10);
    }

    if (fosc == MCP251XFD_FOSC_4MHZ)
        dev->sysclk = MCP251XFD_FOSC_40MHZ;
    else
        dev->sysclk = fosc;

    return MCP251XFD_RETURN_OK;
}

/**
 * @brief Computes a NBTCFG/DBTCFG register word for the given baud rate.
 *
 * Iterates BRP values 0-255 and selects the combination of TSEG1/TSEG2 that
 * produces an exact bit period while keeping the sample point closest to 80%.
 * All register fields are stored 0-based (actual value - 1) per the datasheet.
 *
 * @param tseg1_max  Maximum TSEG1 actual value (256 nominal, 32 data).
 * @param tseg2_max  Maximum TSEG2 actual value (128 nominal, 16 data).
 * @param sjw_max    Maximum SJW actual value   (128 nominal, 16 data).
 * @return true on success, false if no valid combination exists.
 */
static bool mcp251xfd_calculate_bit_timing(MCP251XFD *dev, can_baudrates_t baud,
                                           uint32_t tseg1_max, uint32_t tseg2_max, uint32_t sjw_max,
                                           uint32_t *btcfg)
{
    uint32_t sysclk = (uint32_t)dev->sysclk;
    uint32_t baud_hz = (uint32_t)baud;

    uint32_t best_brp = 0, best_tseg1 = 0, best_tseg2 = 0;
    uint32_t best_sp_error = UINT32_MAX;
    bool found = false;

    for (uint32_t brp = 0; brp < 256; brp++)
    {
        uint32_t tq_freq = sysclk / (brp + 1);
        if (tq_freq % baud_hz != 0)
            continue;

        uint32_t tq_total = tq_freq / baud_hz; // total TQ per bit = 1(sync) + TSEG1 + TSEG2
        if (tq_total < 3 || tq_total > (1 + tseg1_max + tseg2_max))
            continue;

        // Target 80% sample point: TSEG2_actual ≈ tq_total × 0.20
        uint32_t tseg2 = (tq_total + 4) / 5;
        if (tseg2 < 1)
            tseg2 = 1;
        if (tseg2 > tseg2_max)
            tseg2 = tseg2_max;

        uint32_t tseg1 = tq_total - 1 - tseg2;
        if (tseg1 < 1 || tseg1 > tseg1_max)
            continue;

        // Actual sample point in tenths of a percent (0–1000).
        uint32_t sp = ((1 + tseg1) * 1000) / tq_total;
        uint32_t sp_error = sp > 800 ? sp - 800 : 800 - sp;

        if (!found || sp_error < best_sp_error)
        {
            best_sp_error = sp_error;
            best_brp = brp;
            best_tseg1 = tseg1;
            best_tseg2 = tseg2;
            found = true;
        }
    }

    if (!found)
        return false;

    uint32_t sjw = best_tseg2 < sjw_max ? best_tseg2 : sjw_max;

    // BRP, TSEG1, TSEG2, SJW are all stored as (actual − 1) in the register.
    *btcfg = ((best_brp) << 24) |
             ((best_tseg1 - 1) << 16) |
             ((best_tseg2 - 1) << 8) |
             ((sjw - 1) << 0);

    return true;
}

mcp251xfd_return_t mcp251xfd_set_baudrates(MCP251XFD *dev, can_baudrates_t nominal_baud, can_baudrates_t data_baud)
{
    CHECK_NULL_PARAM(dev);

    if (nominal_baud == 0 || data_baud == 0)
    {
        errorf("Invalid CAN baud rates.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    uint32_t nbtcfg, dbtcfg;

    // Nominal phase: TSEG1 ≤ 256, TSEG2 ≤ 128, SJW ≤ 128 (CINBTCFG datasheet limits).
    if (!mcp251xfd_calculate_bit_timing(dev, nominal_baud, 256, 128, 128, &nbtcfg))
    {
        errorf("No valid nominal bit timing for selected baud rate and clock.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    // Data phase: TSEG1 ≤ 32, TSEG2 ≤ 16, SJW ≤ 16 (CIDBTCFG datasheet limits).
    if (!mcp251xfd_calculate_bit_timing(dev, data_baud, 32, 16, 16, &dbtcfg))
    {
        errorf("No valid data bit timing for selected baud rate and clock.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    mcp251xfd_write_word(dev, MCP251XFD_REG_CINBTCFG, nbtcfg);
    mcp251xfd_write_word(dev, MCP251XFD_REG_CIDBTCFG, dbtcfg);

    return MCP251XFD_RETURN_OK;
}

uint8_t mcp251xfd_get_fifo_ram_usage(const mcp251xfd_fifo_config_t *config)
{
    return config->depth * (8 + (uint8_t)config->payload);
}

mcp251xfd_return_t mcp251xfd_configure_fifo(MCP251XFD *dev, uint8_t fifo_num, const mcp251xfd_fifo_config_t *config, uint32_t *ram_used)
{
    CHECK_NULL_PARAM(dev);
    CHECK_NULL_PARAM(config);

    if (fifo_num < 1 || fifo_num > 31)
    {
        errorf("FIFO number must be 1-31.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    if (config->depth < 1 || config->depth > 32)
    {
        errorf("FIFO depth must be 1-32.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    if (config->tx && config->tx_priority > 31)
    {
        errorf("TX priority must be 0-31.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    uint8_t plsize;
    switch (config->payload)
    {
    case MCP251XFD_PLSIZE_8:
        plsize = 0;
        break;
    case MCP251XFD_PLSIZE_12:
        plsize = 1;
        break;
    case MCP251XFD_PLSIZE_16:
        plsize = 2;
        break;
    case MCP251XFD_PLSIZE_20:
        plsize = 3;
        break;
    case MCP251XFD_PLSIZE_24:
        plsize = 4;
        break;
    case MCP251XFD_PLSIZE_32:
        plsize = 5;
        break;
    case MCP251XFD_PLSIZE_48:
        plsize = 6;
        break;
    case MCP251XFD_PLSIZE_64:
        plsize = 7;
        break;
    default:
        errorf("Invalid FIFO payload size.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    uint32_t fifocon = 0;

    fifocon |= ((config->depth - 1) << MCP251XFD_FIFOCON_FSIZE_SFT) & MCP251XFD_FIFOCON_FSIZE_MASK;
    fifocon |= (plsize << MCP251XFD_FIFOCON_PLSIZE_SFT) & MCP251XFD_FIFOCON_PLSIZE_MASK;
    fifocon |= MCP251XFD_FIFOCON_FRESET;

    if (config->tx)
    {
        fifocon |= MCP251XFD_FIFOCON_TXEN;
        fifocon |= (config->tx_priority << MCP251XFD_FIFOCON_TXPRI_SFT) & MCP251XFD_FIFOCON_TXPRI_MASK;
        if (config->auto_rtr)
            fifocon |= MCP251XFD_FIFOCON_RTREN;
    }

    // Each message object = 8-byte header (T0+T1) + payload bytes.
    if (ram_used)
        *ram_used = mcp251xfd_get_fifo_ram_usage(config);

    // FIFO register macro is 0-based from FIFO 1; fifo_num is 1-based.
    mcp251xfd_write_word(dev, MCP251XFD_REG_FIFOCON(fifo_num - 1), fifocon);

    return MCP251XFD_RETURN_OK;
}

/**
 * @brief Packs a CAN ID into the hardware register word format used by CiFLTOBJn, CiMASKn, and message object T0.
 * Layout: SID[10:0] at bits 10:0, EID[17:0] at bits 28:11, IDE/EXIDE at bit 30.
 *
 * @param id       11-bit standard ID in bits 10:0, or 29-bit extended ID in bits 28:0.
 * @param extended true for a 29-bit extended ID, false for an 11-bit standard ID.
 */
static inline uint32_t mcp251xfd_pack_id(uint32_t id, bool extended)
{
    if (extended)
        return ((id >> 18) & 0x7FF) | ((id & 0x3FFFF) << 11) | (1UL << 30);
    return id & 0x7FF;
}

/**
 * @brief Unpacks a hardware register word back into a flat CAN ID.
 *
 * @param reg      Register word in CiFLTOBJn / message T0 format.
 * @param extended Set to true if the register encodes an extended ID, false for standard.
 * @return         29-bit extended ID in bits 28:0, or 11-bit standard ID in bits 10:0.
 */
static inline uint32_t mcp251xfd_unpack_id(uint32_t reg, bool *extended)
{
    *extended = (reg >> 30) & 1;
    if (*extended)
        return ((reg & 0x7FF) << 18) | ((reg >> 11) & 0x3FFFF);
    return reg & 0x7FF;
}

mcp251xfd_return_t mcp251xfd_configure_filter(MCP251XFD *dev,
                                              uint8_t filter_num,
                                              uint32_t id,
                                              uint32_t mask,
                                              bool extended,
                                              uint8_t fifo_num)
{
    CHECK_NULL_PARAM(dev);

    if (filter_num > 31)
    {
        errorf("Filter number must be 0-31.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    if (fifo_num < 1 || fifo_num > 31)
    {
        errorf("FIFO number must be 1-31.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    // Disable the filter before modifying its registers to avoid partial matches.
    uint16_t fltcon_reg = MCP251XFD_REG_C1FLTCON0 + (filter_num / 4) * 4;
    uint8_t byte_shift = (filter_num % 4) * 8;
    uint32_t fltcon = mcp251xfd_read_word(dev, fltcon_reg);
    fltcon &= ~(0xFFUL << byte_shift);
    mcp251xfd_write_word(dev, fltcon_reg, fltcon);

    // MIDE=1 in the mask enforces that only frames whose IDE bit matches EXIDE are accepted.
    mcp251xfd_write_word(dev, MCP251XFD_REG_FLTOBJ(filter_num), mcp251xfd_pack_id(id, extended));
    mcp251xfd_write_word(dev, MCP251XFD_REG_MASK0(filter_num), mcp251xfd_pack_id(mask, extended) | (1UL << 30));

    // Enable filter: FLTEN (bit 7) | FSEL[4:0] = FIFO number.
    fltcon |= ((1UL << 7) | (fifo_num & 0x1F)) << byte_shift;
    mcp251xfd_write_word(dev, fltcon_reg, fltcon);

    return MCP251XFD_RETURN_OK;
}

mcp251xfd_return_t mcp251xfd_disable_filter(MCP251XFD *dev, uint8_t filter_num)
{
    CHECK_NULL_PARAM(dev);

    if (filter_num > 31)
    {
        errorf("Filter number must be 0-31.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    uint16_t fltcon_reg = MCP251XFD_REG_C1FLTCON0 + (filter_num / 4) * 4;
    uint8_t byte_shift = (filter_num % 4) * 8;

    uint32_t fltcon = mcp251xfd_read_word(dev, fltcon_reg);
    fltcon &= ~(1UL << (byte_shift + 7)); // Clear FLTEN only; preserve FSEL.
    mcp251xfd_write_word(dev, fltcon_reg, fltcon);

    return MCP251XFD_RETURN_OK;
}

mcp251xfd_return_t mcp251xfd_configure_interrupts(MCP251XFD *dev, uint32_t enable_mask)
{
    CHECK_NULL_PARAM(dev);

    // Preserve existing flag bits (bits 15:0) to avoid clearing active interrupts.
    uint32_t ciint = mcp251xfd_read_word(dev, MCP251XFD_REG_CIINT);
    ciint = (ciint & 0x0000FFFF) | ((enable_mask & 0xFFFF) << 16);
    mcp251xfd_write_word(dev, MCP251XFD_REG_CIINT, ciint);

    return MCP251XFD_RETURN_OK;
}

mcp251xfd_return_t mcp251xfd_get_interrupt_flags(MCP251XFD *dev, uint32_t *flags)
{
    CHECK_NULL_PARAM(dev);
    CHECK_NULL_PARAM(flags);

    *flags = mcp251xfd_read_word(dev, MCP251XFD_REG_CIINT) & 0xFFFF;

    return MCP251XFD_RETURN_OK;
}

mcp251xfd_return_t mcp251xfd_clear_interrupt_flags(MCP251XFD *dev, uint32_t clear_mask)
{
    CHECK_NULL_PARAM(dev);

    // Writing 0 to a flag bit clears it; preserve enable bits and any flags not in clear_mask.
    uint32_t ciint = mcp251xfd_read_word(dev, MCP251XFD_REG_CIINT);
    ciint &= ~(clear_mask & 0xFFFF);
    mcp251xfd_write_word(dev, MCP251XFD_REG_CIINT, ciint);

    return MCP251XFD_RETURN_OK;
}

/**
 * @brief Zero initialises the internal RAM of the MCP251xFD device for ECC operation.
 *
 * @param dev The MCP251xFD device instance.
 */
static void mcp251xfd_initialise_ram(MCP251XFD *dev)
{
    uint8_t zero_data[4] = {0}; // Buffer of zeros to write to RAM.

    for (uint16_t addr = MCP251XFD_RAM_START; addr <= MCP251XFD_RAM_END - 3; addr += 4)
    {
        mcp251xfd_write_register(dev, addr, zero_data, 4);
    }
}

mcp251xfd_return_t mcp251xfd_initialise(MCP251XFD *dev, mcp251xfd_config_t *config)
{
    /// Firstly, validate the provided parameters.

    // Device is null
    CHECK_NULL_PARAM(dev);

    // Configuration pointer is null
    CHECK_NULL_PARAM(config);

    // Function pointers are null
    CHECK_NULL_PARAM(config->elapsed_us);
    CHECK_NULL_PARAM(config->delay_func);
    CHECK_NULL_PARAM(config->chip_enable);
    CHECK_NULL_PARAM(config->spi_transfer);

    /// Validate configuration parameters.
    if (config->fosc == 0)
    {
        errorf("Invalid external clock frequency selection in configuration.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    if (config->nominal_baud == 0 || config->data_baud == 0)
    {
        errorf("Invalid CAN baud rate selection in configuration.");
        return MCP251XFD_RETURN_INVALID_PARAM;
    }

    /// Set parameters.
    dev->time_us = config->elapsed_us;
    dev->delay = config->delay_func;
    dev->chip_enable = config->chip_enable;
    dev->spi_transfer = config->spi_transfer;
    dev->config = *config; // Copy entire configuration struct.

    dev->initialised = false;

    /// Reset device.
    mcp251xfd_reset_device(dev);

    /// Configure and bring up oscillator based on selected external clock frequency.
    if (mcp251xfd_configure_osc(dev, config->fosc) != MCP251XFD_RETURN_OK)
    {
        errorf("Failed to configure oscillator.");
        return MCP251XFD_RETURN_ERROR;
    }

    /// Set nominal and data bit timings.
    if (mcp251xfd_set_baudrates(dev, config->nominal_baud, config->data_baud) != MCP251XFD_RETURN_OK)
    {
        errorf("Failed to set bit timings.");
        return MCP251XFD_RETURN_ERROR;
    }

    if (config->enable_ecc)
    {
        mcp251xfd_initialise_ram(dev);
        // Enable ECC with 1-bit correction and 2-bit detection capability.
        mcp251xfd_write_word(dev, MCP251XFD_REG_ECCCON, MCP251XFD_ECCCON_ECCEN | MCP251XFD_ECCCON_SECIE | MCP251XFD_ECCCON_DEDIE);
    }
    else
    {
        // Disable ECC to reduce latency.
        mcp251xfd_write_word(dev, MCP251XFD_REG_ECCCON, 0x00);
    }

    /// Configure FIFOs.
    if (config->fifo_count > 0)
    {
        CHECK_NULL_PARAM(config->fifo_configs);

        if (config->fifo_count > 31)
        {
            errorf("fifo_count exceeds maximum of 31.");
            return MCP251XFD_RETURN_INVALID_PARAM;
        }

        // Pre-flight RAM check before touching any registers.
        uint32_t total_ram = 0;
        const uint32_t ram_available = MCP251XFD_RAM_END - MCP251XFD_RAM_START + 1;
        for (uint8_t i = 0; i < config->fifo_count; i++)
            total_ram += mcp251xfd_get_fifo_ram_usage(&config->fifo_configs[i]);

        if (total_ram > ram_available)
        {
            errorf("FIFO configuration requires %u bytes but only %u are available.", total_ram, ram_available);
            return MCP251XFD_RETURN_INVALID_PARAM;
        }

        for (uint8_t i = 0; i < config->fifo_count; i++)
        {
            mcp251xfd_return_t result = mcp251xfd_configure_fifo(dev, i + 1, &config->fifo_configs[i], NULL);
            if (result != MCP251XFD_RETURN_OK)
            {
                errorf("Failed to configure FIFO %u.", i + 1);
                return result;
            }
        }
    }
    else
    {
        // If no FIFOs configured enable TX queue and one RX FIFO at maximum depth with 64-byte payload.

        // Enable TX queue in CiCON (config mode only).
        uint32_t cicon = mcp251xfd_read_word(dev, MCP251XFD_REG_CICON);
        mcp251xfd_write_word(dev, MCP251XFD_REG_CICON, cicon | MCP251XFD_CICON_TXQEN);

        // TX queue: depth=32 (FSIZE=31), payload=64 (PLSIZE=7), reset head/tail.
        uint32_t txqcon = (31 << MCP251XFD_FIFOCON_FSIZE_SFT) |
                          (7 << MCP251XFD_FIFOCON_PLSIZE_SFT) |
                          MCP251XFD_FIFOCON_FRESET;
        mcp251xfd_write_word(dev, MCP251XFD_REG_CITXQCON, txqcon);

        // FIFO 1 as RX: depth=32, payload=64.
        mcp251xfd_fifo_config_t default_rx = {
            .tx = false,
            .depth = 32,
            .payload = MCP251XFD_PLSIZE_64,
        };
        mcp251xfd_configure_fifo(dev, 1, &default_rx, NULL);
    }

    dev->initialised = true;
    return MCP251XFD_RETURN_OK;
}

mcp251xfd_return_t mcp251xfd_deinitialise(MCP251XFD *dev)
{
    CHECK_NULL_PARAM(dev);

    if (!dev->initialised)
        return MCP251XFD_RETURN_OK;

    mcp251xfd_return_t ret = mcp251xfd_request_opmode(dev, MCP251XFD_OPMODE_SLEEP);
    if (ret != MCP251XFD_RETURN_OK)
        return ret;

    dev->initialised = false;
    return MCP251XFD_RETURN_OK;
}

mcp251xfd_return_t mcp251xfd_reset(MCP251XFD *dev)
{
    CHECK_NULL_PARAM(dev);

    return mcp251xfd_initialise(dev, &dev->config);
}