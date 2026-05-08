/**
 * @file mcp2518fd.h
 * @brief Interface for the MicroChip MCP251xFD family of external CAN controller drivers.
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

/* Error handling */
const char *mcp251xfd_get_error_msg(void);

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
 * @struct mcp251xfd_config
 * @brief Configuration structure for initializing the MCP251xFD device.
 *        This structure should be populated with the desired configuration parameters before calling the initialization function.
 */
typedef struct mcp251xfd_config
{
    // Function pointer for a delay function, used for timing requirements during initialization and operation.
    void (*delay_func)(uint32_t microseconds);

    // Function pointer for controlling the chip enable (CE) pin of the MCP251xFD device.
    void (*chip_enable)(void *iface, bool enable);

    // Function pointer for performing SPI data transfers with the MCP251xFD device.
    void (*spi_transfer)(void *iface,            // Interface pointer, pass context of spi hardware.
                         const uint8_t *tx_data, // Transmit data buffer.
                         uint8_t *rx_data,       // Recieve data buffer.
                         size_t length);         // Length of the data to be transferred in bytes.

} mcp251xfd_config_t;

/**
 * @brief Initializes the MCP251xFD device with the provided configuration.
 * The MCP251xFD chip will be in a known state after this function call.
 *
 * @param dev The MCP251xFD device instance.
 * @param config The configuration parameters.
 *
 * @return true if initialization is successful, false otherwise. Check mcp251xfd_get_error_msg() for more details.
 */
bool mcp251xfd_initialise(MCP251XFD *dev, mcp251xfd_config_t *config);

/**
 * @brief Resets the MCP251xFD device.
 * This resets the device and configures it to the previously set parameters used at initialization.
 *
 * @param dev The MCP251xFD device instance.
 */
void mcp251xfd_reset(MCP251XFD *dev);

#endif // __MCP251XFD_H__