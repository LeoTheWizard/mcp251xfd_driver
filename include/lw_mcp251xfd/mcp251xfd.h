/**
 * @file mcp2518fd.h
 * @brief Driver interface for the MicroChip MCP251xFD family of external SPI CAN controllers.
 *
 * @details This module provides the ability to drive the CAN controller over SPI, allowing transmission of CAN frames on a CAN bus.
 * The MCP251xFD family of controllers support both CAN 2.0 and CAN FD frames, and this driver is designed to support both frame types.
 * This driver is dependent on an SPI bus and requires functions for chip enable control and SPI data transfer as well as timing functions.
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

#include "mcp251xfd_conf.h"
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

    MCP251XFD_RETURN_TIMEOUT,       // A timeout occurred while waiting for an operation to complete.
    MCP251XFD_RETURN_TX_FIFO_FULL,  // The transmit FIFO is full and cannot accept new frames for transmission.
    MCP251XFD_RETURN_RX_FIFO_EMPTY, // The receive FIFO is empty and there are no frames available to read.
    MCP251XFD_RETURN_CRC_ERROR      // CRC mismatch detected in a CRC-protected SPI read.
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
 * @brief Returns whether a CRC read error has occurred since the flag was last cleared.
 *
 * Only meaningful when the device was initialised with config.use_crc = true. A CRC
 * read that still mismatches after MCP251XFD_CRC_RETRIES attempts latches this flag;
 * the (unverified) data is still returned to the caller on a best-effort basis.
 *
 * @param dev   The MCP251xFD device instance.
 * @param clear If true, clears the sticky flag after reading it.
 * @return true if a CRC read error has been latched, false otherwise.
 */
bool mcp251xfd_get_spi_crc_error(MCP251XFD *dev, bool clear);

/**
 * @enum mcp251xfd_fosc
 * @brief Enumeration of supported external clock frequencies for the MCP251xFD device.
 */
typedef enum mcp251xfd_fosc
{
    MCP251XFD_FOSC_4MHZ = 4000000,
    MCP251XFD_FOSC_20MHZ = 20000000,
    MCP251XFD_FOSC_40MHZ = 40000000,
} mcp251xfd_fosc_t;

/**
 * @enum mcp251xfd_plsize
 * @brief Payload size options for a CAN message object stored in a FIFO.
 */
typedef enum mcp251xfd_plsize
{
    MCP251XFD_PLSIZE_8 = 8,
    MCP251XFD_PLSIZE_12 = 12,
    MCP251XFD_PLSIZE_16 = 16,
    MCP251XFD_PLSIZE_20 = 20,
    MCP251XFD_PLSIZE_24 = 24,
    MCP251XFD_PLSIZE_32 = 32,
    MCP251XFD_PLSIZE_48 = 48,
    MCP251XFD_PLSIZE_64 = 64,
} mcp251xfd_plsize_t;

/**
 * @struct mcp251xfd_fifo_config
 * @brief Configuration for a single FIFO (1–31).
 */
typedef struct mcp251xfd_fifo_config
{
    bool tx;             // true = transmit FIFO, false = receive FIFO.
    uint8_t depth;       // Number of message objects: 1–32.
    uint8_t payload;     // Payload bytes reserved per object. Use mcp251xfd_plsize_t values.
    uint8_t tx_priority; // Arbitration priority 0–31, TX FIFOs only.
    bool auto_rtr;       // Auto-respond to remote frames, TX FIFOs only.
} mcp251xfd_fifo_config_t;

/**
 * @brief Calculates the RAM usage in bytes for a FIFO configuration.
 */
uint32_t mcp251xfd_get_fifo_ram_usage(const mcp251xfd_fifo_config_t *config);

/**
 * @struct mcp251xfd_config
 * @brief Configuration structure for initialising the MCP251xFD device.
 *        This structure should be populated with the desired configuration parameters before calling the initialisation function.
 */
typedef struct mcp251xfd_config
{
    // Function pointer for a microsecond-resolution timer function, used for timing requirements during initialisation and operation.
    uint32_t (*elapsed_us)(void);

    // Function pointer for a delay function, used for timing requirements during initialisation and operation.
    void (*delay_func)(uint32_t microseconds);

    // Function pointer for controlling the chip enable (CE) pin of the MCP251xFD device.
    void (*chip_enable)(void *iface, bool enable);

    // Function pointer for performing SPI data transfers with the MCP251xFD device.
    void (*spi_transfer)(void *iface,            // Interface pointer, pass context of spi hardware.
                         const uint8_t *tx_data, // Transmit data buffer.
                         uint8_t *rx_data,       // Recieve data buffer.
                         size_t length);         // Length of the data to be transferred in bytes.

    // Opaque pointer passed verbatim as the first argument to chip_enable and spi_transfer.
    // Use this to carry your SPI peripheral handle, GPIO port, or any other hardware context.
    void *iface;

    // External clock frequency selection for the MCP251xFD device. Select the correct frequency based on your board design.
    uint32_t fosc; // Use mcp251xfd_fosc_t values.

    // Initial CAN bus baud rate for the nominal bit timing phase. Used during arbitration and control frames.
    uint32_t nominal_baud; // Use can_baudrates_t values.

    // Initial CAN bus baud rate for the data bit timing phase. Used during CAN FD data frames when BRS is enabled.
    uint32_t data_baud; // Use can_baudrates_t values.

    // Enable or disable ECC error correction for the internal RAM of the MCP251xFD device.
    bool enable_ecc;

    // Route all register/RAM access through CRC-protected SPI commands once the
    // oscillator is up. Failed reads are retried (MCP251XFD_CRC_RETRIES) and, on
    // persistent failure, latch a sticky error queryable via mcp251xfd_get_spi_crc_error().
    // Reset and oscillator bring-up always use plain SPI regardless of this setting.
    bool use_crc;

    // Chip model. Set to MODEL_MCP2518FD or MODEL_MCP2517FD to match your hardware.
    uint8_t model; // Use mcp251xfd_model_t values.

    // FIFO configurations.
    mcp251xfd_fifo_config_t *fifo_configs;
    uint8_t fifo_count;

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
mcp251xfd_return_t mcp251xfd_initialise(MCP251XFD *dev,
                                        mcp251xfd_config_t *config);

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
 *  OPERATING MODE.
 */

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
mcp251xfd_return_t mcp251xfd_request_opmode(MCP251XFD *dev,
                                            mcp251xfd_opmode_t opmode);

/**
 * @brief Awaits for the MCP251xFD device to enter the previously requested operational mode.
 *
 * @param dev The MCP251xFD device instance.
 * @param mode The previously requested operational mode.
 * @param timeout_us The timeout in microseconds.
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_await_opmode(MCP251XFD *dev,
                                          mcp251xfd_opmode_t mode,
                                          uint32_t timeout_us);

/**
 * @brief Requests change of operating mode and waits for the device to enter that mode.
 * Combines mcp251xfd_request_opmode() and mcp251xfd_await_opmode() into a single function for convenience.
 *
 * @param dev The MCP251xFD device instance.
 * @param mode The desired operational mode to change to.
 * @param timeout_us The timeout in microseconds to wait for the device to enter the requested mode.
 *
 * @return mcp251xfd_return_t indicating the result of the operation, including timeout if the device fails to enter the requested mode within the specified time.
 */
mcp251xfd_return_t mcp251xfd_change_opmode(MCP251XFD *dev,
                                           mcp251xfd_opmode_t mode,
                                           uint32_t timeout_us);

/**
 * @brief Gets the current operational mode of the MCP251xFD device.
 *
 * @param dev The MCP251xFD device instance.
 * @param mode Pointer to store the current operational mode.
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_get_opmode(MCP251XFD *dev,
                                        mcp251xfd_opmode_t *mode);

/**
 * @brief Sets the nominal and data bit timings for the CAN bus communication.
 *
 * @param dev The MCP251xFD device instance.
 * @param nominal_baud The desired nominal baud rate for the arbitration phase of CAN communication.
 * @param data_baud The desired data baud rate for the data phase of CAN FD communication.
 *
 * @return mcp251xfd_return_t indicating the result of the operation, including error if the baud rates are invalid or if the device is not in a mode that allows changing bit timings.
 */
mcp251xfd_return_t mcp251xfd_set_baudrates(MCP251XFD *dev,
                                           can_baudrates_t nominal_baud,
                                           can_baudrates_t data_baud);

/**
 * @struct mcp251xfd_bit_timing
 * @brief Bit-timing segments for one CAN phase, as actual time-quanta counts
 *        (not register-encoded — the driver subtracts 1 per field when packing).
 *
 * These map one-to-one onto Linux SocketCAN's struct can_bittiming:
 *   brp   = bittiming.brp
 *   tseg1 = bittiming.prop_seg + bittiming.phase_seg1
 *   tseg2 = bittiming.phase_seg2
 *   sjw   = bittiming.sjw
 */
typedef struct mcp251xfd_bit_timing
{
    uint16_t brp;   /**< Baud rate prescaler: 1-256. */
    uint16_t tseg1; /**< Time segment 1 (prop + phase 1): 1-256 nominal, 1-32 data. */
    uint8_t  tseg2; /**< Time segment 2 (phase 2): 1-128 nominal, 1-16 data. */
    uint8_t  sjw;   /**< Sync jump width: 1-128 nominal, 1-16 data; must be <= tseg2. */
} mcp251xfd_bit_timing_t;

/**
 * @brief Sets nominal (and optionally data) bit timing from explicit segment values.
 *
 * Use this instead of mcp251xfd_set_baudrates() when the caller computes its own
 * bit timing rather than a target baud rate — e.g. a Linux SocketCAN driver, where
 * the kernel derives the segments from the requested bitrate and controller clock.
 * Writes CINBTCFG (and CIDBTCFG if @p data is non-NULL); call in configuration mode.
 *
 * @param dev     The MCP251xFD device instance.
 * @param nominal Nominal (arbitration) phase segments. Required.
 * @param data    Data (CAN FD) phase segments, or NULL to leave CIDBTCFG unchanged.
 *
 * @return MCP251XFD_RETURN_OK on success, MCP251XFD_RETURN_INVALID_PARAM if any
 *         segment is out of range for its phase, or a null required parameter.
 */
mcp251xfd_return_t mcp251xfd_set_bit_timing(MCP251XFD *dev,
                                            const mcp251xfd_bit_timing_t *nominal,
                                            const mcp251xfd_bit_timing_t *data);

/**
 * FIFO and Filter Configuration
 *
 */
#pragma region FIFO and Filter Configuration

/**
 * @brief Configures a FIFO message object.
 *
 * @param dev       The MCP251xFD device instance.
 * @param fifo_num  FIFO number to configure (1–31).
 * @param config    FIFO configuration parameters.
 * @param ram_used  If non-NULL, set to the number of RAM bytes consumed by this FIFO.
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_configure_fifo(MCP251XFD *dev,
                                            uint8_t fifo_num,
                                            const mcp251xfd_fifo_config_t *config,
                                            uint32_t *ram_used);

/**
 * @brief Configures a filter for a receive FIFO.
 *
 * @param dev The MCP251xFD device instance.
 * @param filter_num Filter number to configure (0–31).
 * @param id The identifier to filter for. For extended 29-bit IDs, the ID should be left-aligned in the 32-bit value.
 * @param mask The mask to apply to the ID for filtering. For extended 29-bit IDs, the mask should be left-aligned in the 32-bit value.
 * @param extended True if the filter is for extended 29-bit IDs, false for standard 11-bit IDs.
 * @param fifo_num The receive FIFO number (1–31) that this filter should be applied to.
 *
 * @return mcp251xfd_return_t indicating the result of the operation, including error if the filter number or FIFO number is invalid,
 * or if the device is not in a mode that allows configuring filters.
 */
mcp251xfd_return_t mcp251xfd_configure_filter(MCP251XFD *dev,
                                              uint8_t filter_num,
                                              uint32_t id,
                                              uint32_t mask,
                                              bool extended,
                                              uint8_t fifo_num);

/**
 * @brief Disables a previously configured acceptance filter.
 * Clears the FLTEN bit in CiFLTCONn without disturbing the stored ID or mask,
 * so the filter can be re-enabled later by calling mcp251xfd_configure_filter() again.
 *
 * @param dev        The MCP251xFD device instance.
 * @param filter_num The filter number to disable (0–31).
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_disable_filter(MCP251XFD *dev, uint8_t filter_num);

#pragma endregion

/**
 * Interrupts
 */
#pragma region Interrupts

/**
 * @enum mcp251xfd_interrupt_flags
 * @brief Bitmask values for each interrupt source in the MCP251xFD device.
 *
 * Values correspond to the flag bit positions in CiINT[15:0].
 * configure_interrupts() shifts these left 16 to reach the enable bits (CiINT[31:16]).
 * OR multiple values together to address more than one source at a time.
 */
typedef enum mcp251xfd_interrupt_flags
{
    MCP251XFD_INT_TX = (1 << 0),   // TXIF/TXIE   — TX FIFO has a pending interrupt. Read CiTXIF to identify which FIFO.
    MCP251XFD_INT_RX = (1 << 1),   // RXIF/RXIE   — RX FIFO has a pending interrupt. Read CiRXIF to identify which FIFO.
    MCP251XFD_INT_TBC = (1 << 2),  // TBCIF/TBCIE — time base counter overflowed.
    MCP251XFD_INT_MODE = (1 << 3), // MODIF/MODIE — operating mode change completed.
    MCP251XFD_INT_TEF = (1 << 4),  // TEFIF/TEFIE — transmit event FIFO has entries to read.
    // bits 5-7 are unimplemented in hardware.
    MCP251XFD_INT_ECC = (1 << 8),         // ECCIF/ECCIE   — ECC error in RAM. Read CiECCSTA for address.
    MCP251XFD_INT_SPI_CRC = (1 << 9),     // SPICRCIF/SPICRCIE — SPI CRC mismatch on a CRC-protected command.
    MCP251XFD_INT_TX_ATTEMPT = (1 << 10), // TXATIF/TXATIE — TX attempt interrupt (arbitration loss or abort). Read CiTXATIF.
    MCP251XFD_INT_RX_OVFLOW = (1 << 11),  // RXOVIF/RXOVIE — RX FIFO overflowed; at least one frame was dropped. Read CiRXOVIF.
    MCP251XFD_INT_SYS_ERROR = (1 << 12),  // SERRIF/SERRIE — system error (MCP2518FD only; always 0 on MCP2517FD).
    MCP251XFD_INT_CAN_ERROR = (1 << 13),  // CERRIF/CERRIE — CAN bus error; TEC/REC threshold crossed or bus error detected.
    MCP251XFD_INT_WAKE = (1 << 14),       // WAKIF/WAKIE  — CAN bus activity detected while in sleep mode.
    MCP251XFD_INT_INVALID = (1 << 15),    // IVMIF/IVMIE  — invalid message received (DLC > payload size, or malformed FD frame).
} mcp251xfd_interrupt_flags_t;

/**
 * @brief Configures the interrupt enables for the MCP251xFD device.
 * The enable_mask is a bitmask where each bit corresponds to a specific interrupt source.
 *
 * @param dev The MCP251xFD device instance.
 * @param enable_mask Bitmask of interrupts to enable.
 *
 * @return mcp251xfd_return_t indicating the result of the operation, including error if the device is not in a mode that allows configuring interrupts.
 */
mcp251xfd_return_t mcp251xfd_configure_interrupts(MCP251XFD *dev,
                                                  uint32_t enable_mask);

/**
 * @brief Retrieves the current active interrupt flags from the MCP251xFD device.
 *
 * @param dev The MCP251xFD device instance.
 * @param flags Pointer to store the retrieved interrupt flags.
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_get_interrupt_flags(MCP251XFD *dev,
                                                 uint32_t *flags);

/**
 * @brief Clears the specified interrupt flags from the MCP251xFD device.
 *
 * @param dev The MCP251xFD device instance.
 * @param clear_mask Bitmask of interrupts to clear.
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_clear_interrupt_flags(MCP251XFD *dev,
                                                   uint32_t clear_mask);

#pragma endregion

/**
 * CAN Message Transmission and Reception
 *
 *
 */
#pragma region CAN Message Transmission and Reception

/**
 * @brief Transmits a CAN frame using the specified transmit FIFO.
 *
 * @param dev The MCP251xFD device instance.
 * @param fifo_num The transmit FIFO number to use for transmission (1–31).
 * @param frame The CAN frame to be transmitted.
 *
 * @return mcp251xfd_return_t indicating the result of the transmission request, including error if the FIFO is full or if the device is not in a mode that allows transmission.
 */
mcp251xfd_return_t mcp251xfd_transmit(MCP251XFD *dev,
                                      uint8_t fifo_num,
                                      const can_frame_t *frame);

/**
 * @brief Aborts any pending transmissions in the specified transmit FIFO.
 *
 * @param dev The MCP251xFD device instance.
 * @param fifo_num The transmit FIFO number to abort (1–31).
 *
 * @return mcp251xfd_return_t indicating the result of the abort request, including error if the FIFO number is invalid or if the device is not in a mode that allows aborting transmissions.
 */
mcp251xfd_return_t mcp251xfd_abort_tx(MCP251XFD *dev,
                                      uint8_t fifo_num);

/**
 * @brief Checks the number of received CAN frames pending in the specified receive FIFO.
 *
 * @param dev The MCP251xFD device instance.
 * @param fifo_num The receive FIFO number to check (1–31).
 * @param count Pointer to store the number of pending frames in the FIFO.
 *
 * @return mcp251xfd_return_t indicating the result of the operation, including error if the FIFO number is invalid or if the device is not in a mode that allows checking received frames.
 */
mcp251xfd_return_t mcp251xfd_rx_pending(MCP251XFD *dev,
                                        uint8_t fifo_num,
                                        uint8_t *count);

/**
 * @brief Retrieves a single received CAN frame from the specified receive FIFO.
 *
 * @param dev The MCP251xFD device instance.
 * @param fifo_num The receive FIFO number to read from (1–31).
 * @param frame Pointer to store the retrieved CAN frame.
 *
 * @return mcp251xfd_return_t indicating the result of the operation, including error if the FIFO is empty or if the device is not in a mode that allows receiving frames.
 */
mcp251xfd_return_t mcp251xfd_get_received(MCP251XFD *dev,
                                          uint8_t fifo_num,
                                          can_frame_t *frame);

/**
 * @brief Retrieves all pending received CAN frames from the specified receive FIFO, up to the provided buffer size.
 *
 * @param dev The MCP251xFD device instance.
 * @param fifo_num The receive FIFO number to read from (1–31).
 * @param frames Pointer to the buffer to store the retrieved CAN frames.
 * @param max_frames The maximum number of frames to retrieve, corresponding to the size of the provided buffer.
 * @param frame_count Pointer to store the number of retrieved frames.
 *
 * @return mcp251xfd_return_t indicating the result of the operation, including error if the FIFO is empty or if the device is not in a mode that allows receiving frames.

 */
mcp251xfd_return_t mcp251xfd_get_all_received(MCP251XFD *dev,
                                              uint8_t fifo_num,
                                              can_frame_t *frames,
                                              uint8_t max_frames,
                                              uint8_t *frame_count);

/**
 * @brief Discards all pending frames in a receive FIFO and resets its head/tail pointers.
 * Use before a mode change or after a bus error to ensure stale data is not processed.
 *
 * @param dev      The MCP251xFD device instance.
 * @param fifo_num The receive FIFO number to flush (1–31).
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_flush_rx(MCP251XFD *dev,
                                      uint8_t fifo_num);

/**
 * @brief Checks whether a receive FIFO has overflowed and silently dropped at least one frame.
 * Reads and clears the RXOVIF flag in CiFIFOSTAn. Call periodically in a receive loop to detect data loss.
 *
 * @param dev        The MCP251xFD device instance.
 * @param fifo_num   The receive FIFO number to check (1–31).
 * @param overflowed Set to true if an overflow has occurred since the last call, false otherwise.
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_get_rx_overflow(MCP251XFD *dev,
                                             uint8_t fifo_num,
                                             bool *overflowed);

#pragma endregion

/**
 * CAN Error Handling
 *
 * Error state is read from CiTREC (0x0034), which holds the transmit and receive
 * error counters and the derived bus-state flags defined by ISO 11898-1.
 */
#pragma region CAN Error Handling

/**
 * @struct mcp251xfd_error_state
 * @brief Decoded contents of the CiTREC register.
 */
typedef struct mcp251xfd_error_state
{
    uint8_t rec;     // Receive error counter (0–255).
    uint8_t tec;     // Transmit error counter (0–255).
    bool error_warn; // EWARN — TEC or REC >= 96.
    bool rx_warn;    // RXWARN — REC >= 96.
    bool tx_warn;    // TXWARN — TEC >= 96.
    bool rx_passive; // RXBP — node is receive error-passive (REC >= 128).
    bool tx_passive; // TXBP — node is transmit error-passive (TEC >= 128).
    bool bus_off;    // TXBO — node is bus-off (TEC > 255); no transmission possible.
} mcp251xfd_error_state_t;

/**
 * @brief Reads the current CAN error counters and bus-state flags from CiTREC.
 *
 * @param dev   The MCP251xFD device instance.
 * @param state Pointer to an mcp251xfd_error_state_t to populate.
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_get_error_state(MCP251XFD *dev,
                                             mcp251xfd_error_state_t *state);

/**
 * @brief Initiates bus-off recovery.
 *
 * When the node is bus-off (TXBO set), places the controller back into normal
 * mode and waits for the CAN-mandated recovery sequence (128 × 11 consecutive
 * recessive bits) to complete before returning.
 *
 * Returns immediately with MCP251XFD_RETURN_OK if the node is not currently bus-off.
 *
 * @param dev        The MCP251xFD device instance.
 * @param timeout_us Maximum time in microseconds to wait for recovery.
 *
 * @return mcp251xfd_return_t — OK on recovery, TIMEOUT if the bus does not recover
 *         within timeout_us, ERROR if the mode request fails.
 */
mcp251xfd_return_t mcp251xfd_recover_bus_off(MCP251XFD *dev, uint32_t timeout_us);

#pragma endregion

/**
 * GPIO Control
 *
 * The MCP251xFD exposes two general-purpose I/O pins (GPIO0, GPIO1) controlled
 * via the IOCON register (0x0E04). Each pin can be independently set as an input
 * or output, and switched between GPIO mode and its alternate function:
 *   GPIO0 alternate: INT1 (second interrupt output)
 *   GPIO1 alternate: CLKO (clock output) / SOF (start-of-frame signal)
 *
 * IOCON also controls open-drain mode for the primary TXCAN and INT pins.
 */
#pragma region GPIO Control

/**
 * @enum mcp251xfd_gpio_pin
 * @brief Selects which GPIO pin to operate on.
 */
typedef enum mcp251xfd_gpio_pin
{
    MCP251XFD_GPIO0 = 0, // GPIO0 pin (IOCON TRIS0/LAT0/GPIO0/PM0 bits).
    MCP251XFD_GPIO1 = 1, // GPIO1 pin (IOCON TRIS1/LAT1/GPIO1/PM1 bits).
} mcp251xfd_gpio_pin_t;

/**
 * @enum mcp251xfd_gpio_dir
 * @brief Pin data direction.
 */
typedef enum mcp251xfd_gpio_dir
{
    MCP251XFD_GPIO_OUTPUT = 0, // TRIS = 0 — pin drives the line.
    MCP251XFD_GPIO_INPUT = 1,  // TRIS = 1 — pin samples the line.
} mcp251xfd_gpio_dir_t;

/**
 * @enum mcp251xfd_gpio_mode
 * @brief Selects between GPIO and the pin's alternate hardware function.
 */
typedef enum mcp251xfd_gpio_mode
{
    MCP251XFD_GPIO_MODE_ALT = 0,  // PM = 0 — alternate function (INT1 on GPIO0, CLKO on GPIO1).
    MCP251XFD_GPIO_MODE_GPIO = 1, // PM = 1 — general-purpose I/O.
} mcp251xfd_gpio_mode_t;

/**
 * @brief Sets the mode of a GPIO pin (GPIO vs alternate function).
 * Must be called before set_direction or write/read for the chosen mode to take effect.
 *
 * @param dev  The MCP251xFD device instance.
 * @param pin  The pin to configure.
 * @param mode GPIO or alternate function.
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_gpio_set_mode(MCP251XFD *dev,
                                           mcp251xfd_gpio_pin_t pin,
                                           mcp251xfd_gpio_mode_t mode);

/**
 * @brief Sets the data direction of a GPIO pin (input or output).
 * Only meaningful when the pin is in MCP251XFD_GPIO_MODE_GPIO.
 *
 * @param dev The MCP251xFD device instance.
 * @param pin The pin to configure.
 * @param dir Input or output.
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_gpio_set_direction(MCP251XFD *dev,
                                                mcp251xfd_gpio_pin_t pin,
                                                mcp251xfd_gpio_dir_t dir);

/**
 * @brief Writes a logic level to a GPIO output pin (sets the LAT latch bit).
 * The pin must be configured as an output, otherwise this has no visible effect.
 *
 * @param dev   The MCP251xFD device instance.
 * @param pin   The pin to drive.
 * @param value true = high, false = low.
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_gpio_write(MCP251XFD *dev,
                                        mcp251xfd_gpio_pin_t pin,
                                        bool value);

/**
 * @brief Reads the current logic level of a GPIO pin (reads the GPIO status bit).
 * Valid for both input and output pins — for outputs this reflects the driven level.
 *
 * @param dev   The MCP251xFD device instance.
 * @param pin   The pin to sample.
 * @param value Pointer to store the sampled logic level (true = high, false = low).
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_gpio_read(MCP251XFD *dev,
                                       mcp251xfd_gpio_pin_t pin,
                                       bool *value);

/**
 * @brief Enables or disables open-drain mode on the TXCAN pin (IOCON.TXCANOD).
 * When enabled the TX driver becomes open-drain, requiring an external pull-up.
 * Useful for multi-master wired-AND bus configurations.
 *
 * @param dev    The MCP251xFD device instance.
 * @param enable true = open-drain, false = push-pull (default).
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_gpio_set_tx_open_drain(MCP251XFD *dev,
                                                    bool enable);

/**
 * @brief Enables or disables open-drain mode on the primary INT pin (IOCON.INTOD).
 * When enabled the INT driver becomes open-drain, allowing wire-OR with other interrupt sources.
 *
 * @param dev    The MCP251xFD device instance.
 * @param enable true = open-drain, false = push-pull (default).
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_gpio_set_int_open_drain(MCP251XFD *dev,
                                                     bool enable);

/**
 * @brief Enables or disables the Start-of-Frame (SOF) signal output on the GPIO1/CLKO pin (IOCON.SOF).
 * When enabled GPIO1 outputs a pulse at the start of each CAN frame instead of acting as CLKO or GPIO.
 * GPIO1 must be in alternate mode (MCP251XFD_GPIO_MODE_ALT) for this to have effect.
 *
 * @param dev    The MCP251xFD device instance.
 * @param enable true = output SOF signal, false = CLKO or GPIO (default).
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_gpio_set_sof_output(MCP251XFD *dev,
                                                 bool enable);

#pragma endregion

/**
 * Device Identification
 */

/**
 * @brief Reads the DEVID register and identifies whether the device is an MCP2517FD or MCP2518FD.
 * Also updates the model field on the device instance for use by other driver functions.
 *
 * @param dev   The MCP251xFD device instance.
 * @param model Pointer to store the identified model.
 *
 * @return mcp251xfd_return_t indicating the result of the operation, or ERROR if the device ID is unrecognised.
 */
mcp251xfd_return_t mcp251xfd_get_model(MCP251XFD *dev,
                                       mcp251xfd_model_t *model);

/**
 * Transmit Event FIFO (TEF)
 *
 * The TEF records a brief entry for each successfully transmitted frame without consuming
 * message-RAM payload space. Useful for TX timestamping and bus-load diagnostics.
 */
#pragma region Transmit Event FIFO

/**
 * @struct mcp251xfd_tef_config
 * @brief Configuration for the Transmit Event FIFO.
 */
typedef struct mcp251xfd_tef_config
{
    uint8_t depth;   // Number of TEF entries: 1–32.
    bool timestamps; // Record a hardware timestamp in each TEF entry.
} mcp251xfd_tef_config_t;

/**
 * @struct mcp251xfd_tef_entry
 * @brief A single entry read from the Transmit Event FIFO.
 */
typedef struct mcp251xfd_tef_entry
{
    uint32_t id;        // Frame identifier (11-bit or 29-bit depending on flags).
    uint8_t flags;      // Frame flags (EFF, FDF, BRS, ESI). Use can_frame_flags_t values.
    uint8_t dlc;        // Data length code of the transmitted frame.
    uint16_t timestamp; // Hardware timestamp at transmission; valid only if TEF was configured with timestamps=true.
} mcp251xfd_tef_entry_t;

/**
 * @brief Enables and configures the Transmit Event FIFO.
 * Must be called while the device is in configuration mode.
 *
 * @param dev    The MCP251xFD device instance.
 * @param config TEF configuration parameters.
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_enable_tef(MCP251XFD *dev,
                                        const mcp251xfd_tef_config_t *config);

/**
 * @brief Reads the oldest entry from the Transmit Event FIFO and advances the read pointer.
 *
 * @param dev   The MCP251xFD device instance.
 * @param entry Pointer to store the retrieved TEF entry.
 *
 * @return mcp251xfd_return_t — OK on success, RX_FIFO_EMPTY if the TEF has no entries.
 */
mcp251xfd_return_t mcp251xfd_read_tef(MCP251XFD *dev,
                                      mcp251xfd_tef_entry_t *entry);

#pragma endregion

/**
 * Time Base Counter
 */

/**
 * @brief Reads the current value of the 32-bit hardware Time Base Counter (CiTBC).
 * The counter must have been enabled via CiTSCON.TBCEN before calling this function.
 *
 * @param dev       The MCP251xFD device instance.
 * @param timestamp Pointer to store the current counter value.
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_get_timestamp(MCP251XFD *dev,
                                           uint32_t *timestamp);

/**
 * Wake-up Filter
 */

/**
 * @enum mcp251xfd_wakeup_filter
 * @brief Selects the CAN bus activity glitch filter applied during sleep mode (CiCON.WFT).
 */
typedef enum mcp251xfd_wakeup_filter
{
    MCP251XFD_WAKEUP_FILTER_T11 = 0, // Wake if T1 + T2 > 5 × TQ.
    MCP251XFD_WAKEUP_FILTER_T1 = 1,  // Wake if T1 > 6 × TQ.
    MCP251XFD_WAKEUP_FILTER_T2 = 2,  // Wake if T2 > 6 × TQ.
    MCP251XFD_WAKEUP_FILTER_T12 = 3, // Wake if T1 + T2 > 10 × TQ.
} mcp251xfd_wakeup_filter_t;

/**
 * @brief Configures the CAN bus wake-up filter used when the device is in sleep mode.
 * When enabled, only bus activity that passes the selected glitch filter will wake the device.
 *
 * @param dev    The MCP251xFD device instance.
 * @param enable true to enable the wake-up filter, false to disable.
 * @param filter Glitch filter time selection (ignored when enable is false).
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_configure_wakeup(MCP251XFD *dev,
                                              bool enable,
                                              mcp251xfd_wakeup_filter_t filter);

/**
 * Bus Diagnostics
 *
 * CiBDIAG0 and CiBDIAG1 accumulate per-phase error counts and per-type error flags
 * since the last read. Reading the registers clears them.
 */
#pragma region Bus Diagnostics

/**
 * @struct mcp251xfd_diagnostics
 * @brief Decoded contents of CiBDIAG0 and CiBDIAG1.
 */
typedef struct mcp251xfd_diagnostics
{
    uint8_t nominal_rx_errors;  // Nominal-phase receive error count since last read.
    uint8_t nominal_tx_errors;  // Nominal-phase transmit error count since last read.
    uint8_t data_rx_errors;     // Data-phase receive error count since last read.
    uint8_t data_tx_errors;     // Data-phase transmit error count since last read.
    uint16_t error_frame_count; // Total error frames detected since last read.
    bool nbit0_err;             // Nominal phase: bit 0 error.
    bool nbit1_err;             // Nominal phase: bit 1 error.
    bool nack_err;              // Nominal phase: ACK error.
    bool nform_err;             // Nominal phase: form error.
    bool nstuff_err;            // Nominal phase: stuff error.
    bool ncrc_err;              // Nominal phase: CRC error.
    bool txbo_err;              // Node entered bus-off since last read.
    bool dbit0_err;             // Data phase: bit 0 error.
    bool dbit1_err;             // Data phase: bit 1 error.
    bool dform_err;             // Data phase: form error.
    bool dstuff_err;            // Data phase: stuff error.
    bool dcrc_err;              // Data phase: CRC error.
    bool esi;                   // ESI flag was set on the last received FD frame.
    bool dlc_mismatch;          // Received DLC exceeded the configured payload size.
} mcp251xfd_diagnostics_t;

/**
 * @brief Reads and clears the bus diagnostic registers (CiBDIAG0 / CiBDIAG1).
 *
 * @param dev  The MCP251xFD device instance.
 * @param diag Pointer to store the decoded diagnostic data.
 *
 * @return mcp251xfd_return_t indicating the result of the operation.
 */
mcp251xfd_return_t mcp251xfd_read_diagnostics(MCP251XFD *dev,
                                              mcp251xfd_diagnostics_t *diag);

#pragma endregion

#endif // __MCP251XFD_H__