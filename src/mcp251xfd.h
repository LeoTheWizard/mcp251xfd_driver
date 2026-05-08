/**
 * @file mcp2518fd.h
 * @brief Interface for the MicroChip MCP251xFD family of external SPI CAN controller drivers.
 *
 * @details This modules provides the ability to drive the CAN controller over SPI, allowing transmission of CAN frames on a CAN bus.
 * The MCP251xFD family of controllers support both CAN 2.0 and CAN FD frames, and this driver is designed to support both frame types.
 * This driver is dependent on an SPI bus and requires functions for chip enable control and SPI data transfer.
 *
 * @ref Datasheet @ https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/External-CAN-FD-Controller-with-SPI-Interface-DS20006027B.pdf
 * @ref Reference Manual @ https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/ReferenceManuals/MCP25XXFD-CAN-FD-Controller-Module-Family-Reference-Manual-DS20005678E.pdf
 *
 * @author Leo Walker
 *
 * @copyright Copyright (c) 2025 Leo Walker. Licensed under the MIT License.
 *            See LICENSE in the root directory for the full license text.
 *
 */

#ifndef __MCP251XFD_H__
#define __MCP251XFD_H__

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "can.h"

/* Error handling */

/**
 * @brief Retrieves the last error message in plain text.
 * @note MCP251XFD_ENABLE_ERROR_MESSAGES must be defined for this function to return meaningful messages.
 * Otherwise, it returns an empty string.
 */
const char *mcp251xfd_get_error_msg(void);

/**
 * @brief Return codes for MCP251xFD driver functions, indicating the result of operations.
 */
typedef enum mcp251xfd_return
{
    MCP251XFD_RETURN_OK,              // Operation was successful.
    MCP251XFD_RETURN_ERROR,           // A generic error occurred.
    MCP251XFD_RETURN_INVALID_PARAM,   // An invalid parameter was provided to a function.
    MCP251XFD_RETURN_NOT_INITIALISED, // The device instance has not been initialised before use.

    MCP251XFD_RETURN_TX_FIFO_FULL, // The transmit FIFO is full and cannot accept new frames for transmission.
    MCP251XFD_RETURN_RX_FIFO_EMPTY // The receive FIFO is empty and there are no frames available to read.
} mcp251xfd_return_t;

/**
 * @enum mcp251xfd_model
 */
typedef enum mcp251xfd_model
{
    MODEL_MCP2517FD,
    MODEL_MCP2518FD
} mcp251xfd_model_t;

/**
 * @struct MCP251XFD
 * @brief Opaque structure representing an instance of the MCP251xFD CAN controller device.
 *        The actual structure definition is hidden in the implementation file (mcp251xfd.c)
 */
typedef struct mcp2518fd_priv MCP251XFD;

/**
 * @brief Allocates memory for a MCP251xFD device instance.
 * @return Pointer to the allocated MCP251xFD instance, or NULL if allocation fails.
 */
MCP251XFD *mcp251xfd_create_instance(void);

/**
 * @brief Frees the memory allocated for a MCP251xFD device instance.
 * @param instance Pointer to the MCP251xFD instance to be freed.
 */
void mcp251xfd_destroy_instance(MCP251XFD *instance);

/**
 * @enum mcp251xfd_fosc
 * @brief Enumeration of supported external clock frequencies for the MCP251xFD device.
 */
typedef enum mcp251xfd_fosc
{
    MCP251XFD_FOSC_4MHZ,
    MCP251XFD_FOSC_20MHZ,
    MCP251XFD_FOSC_40MHZ
} mcp251xfd_fosc_t;

/**
 * @struct mcp251xfd_config
 * @brief Configuration structure for initialising the MCP251xFD device.
 *        This structure should be populated with the desired configuration parameters before calling the initialisation function.
 */
typedef struct mcp251xfd_config
{
    // Function pointer for a delay function, used for timing requirements during initialisation and operation.
    void (*delay_func)(uint32_t microseconds);

    // Function pointer for controlling the chip enable (CE) pin of the MCP251xFD device.
    void (*chip_enable)(void *iface, bool enable);

    // Function pointer for performing SPI data transfers with the MCP251xFD device.
    void (*spi_transfer)(void *iface,            // Interface pointer, pass context of spi hardware.
                         const uint8_t *tx_data, // Transmit data buffer.
                         uint8_t *rx_data,       // Recieve data buffer.
                         size_t length);         // Length of the data to be transferred in bytes.

    // External clock frequency selection for the MCP251xFD device. Select the correct frequency based on your board design.
    mcp251xfd_fosc_t fosc;

    // Initial CAN bus baud rate for the nominal bit timing phase. Used during arbitration and control frames.
    can_baudrates_t nominal_baud;

    // Initial CAN bus baud rate for the data bit timing phase. Used during CAN FD data frames when BRS is enabled.
    can_baudrates_t data_baud;

} mcp251xfd_config_t;

/**
 * @brief Initialises the MCP251xFD device with the provided configuration.
 * The MCP251xFD chip will be in a known state after this function call.
 *
 * @param dev The MCP251xFD device instance.
 * @param config The configuration parameters.
 *
 * @return mcp251xfd_return_t indicating the result of the initialisation process.
 */
mcp251xfd_return_t mcp251xfd_initialise(MCP251XFD *dev, mcp251xfd_config_t *config);

/**
 * @brief Deinitialises the MCP251xFD device, putting it into a low power state.
 *
 * @param dev The MCP251xFD device instance.
 *
 * @return mcp251xfd_return_t indicating the result of the deinitialisation process.
 */
mcp251xfd_return_t mcp251xfd_deinitialise(MCP251XFD *dev);

/**
 * @brief Resets the MCP251xFD device.
 * This resets the device and configures it to the previously set parameters used at initialization.
 *
 * @param dev The MCP251xFD device instance.
 *
 * @return mcp251xfd_return_t indicating the result of the reset process.
 */
mcp251xfd_return_t mcp251xfd_reset(MCP251XFD *dev);

/**
 * @enum mcp251xfd_opmode
 * @brief Enumeration of operational modes for the MCP251xFD device.
 */
typedef enum mcp251xfd_opmode
{
    MCP251XFD_OPMODE_NORMAL,            //< Normal CAN FD mode; supports mixing of CAN FD and Classic CAN 2.0 frames
    MCP251XFD_OPMODE_SLEEP,             //< Low power sleep mode, can be configured to wake on CAN bus activity or external interrupt.
    MCP251XFD_OPMODE_INTERNAL_LOOPBACK, //< Internal loopback mode for self-testing, where transmitted frames are received internally without being sent on the CAN bus.
    MCP251XFD_OPMODE_LISTEN_ONLY,       //< Listen-only mode where the device can receive frames from the CAN bus but does not acknowledge or transmit any frames.
    MCP251XFD_OPMODE_CONFIG,            //< Configuration mode for setting up the device.
    MCP251XFD_OPMODE_EXTERNAL_LOOPBACK, //< External loopback mode for testing, where transmitted frames are received externally without being sent on the CAN bus.
    MCP251XFD_OPMODE_CLASSIC,           //< Classic CAN 2.0 mode.
    MCP251XFD_OPMODE_RESTRICTED         //< Restricted operation mode.
} mcp251xfd_opmode_t;

/**
 * @brief Requests a change in the operational mode of the MCP251xFD device.
 *
 * @param dev The MCP251xFD device instance.
 * @param opmode The desired operational mode.
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_request_opmode(MCP251XFD *dev, mcp251xfd_opmode_t opmode);

#endif // __MCP251XFD_H__