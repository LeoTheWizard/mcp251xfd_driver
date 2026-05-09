/**
 * @file can.h
 * @brief CAN specific definitions and structures.
 * @author Leo Walker
 *
 * @copyright Copyright (c) 2025 Leo Walker. Licensed under the MIT License.
 *            See LICENSE in the root directory for the full license text.
 *
 */

#ifndef __CAN_H__
#define __CAN_H__

#include <stdint.h>

#define CAN_SFF_MASK 0x000007FFUL // Standard Identifier Mask
#define CAN_EFF_MASK 0x1FFFFFFFUL // Extended Identifier Mask

/**
 * @enum can_frame_flags
 * @brief Flags for CAN frame types and features.
 *        These flags can be combined using bitwise OR to indicate multiple features of a CAN frame.
 *        - CAN_FRAME_FLAG_EEF (0x01): Indicates that the frame uses the Extended Frame Format (EFF).
 *        - CAN_FRAME_FLAG_FDF (0x02): Indicates that the frame is a CAN FD frame, which allows for larger data payloads.
 *        - CAN_FRAME_FLAG_BRS (0x04): Indicates that the frame uses Bit Rate Switching (BRS), which allows for a higher data rate during the data phase of a CAN FD frame.
 *        - CAN_FRAME_FLAG_ESI (0x08): Indicates that the frame has the Error State Indicator (ESI) flag set, which is used in CAN FD to indicate the error state of the transmitting node.
 */
typedef enum can_frame_flags : uint8_t
{
    CAN_FRAME_FLAG_EEF = 0x01, // Extended Frame Format (EFF) flag
    CAN_FRAME_FLAG_FDF = 0x02, // CAN FD Frame (FDF) flag
    CAN_FRAME_FLAG_BRS = 0x04, // Bit Rate Switch (BRS) flag
    CAN_FRAME_FLAG_ESI = 0x08  // Error State Indicator (ESI) flag
} can_frame_flags_t;

/**
 * @struct can_frame
 * @brief Represents a CAN frame, which can be either a standard or extended frame, and may also be a CAN FD frame.
 */
typedef struct can_frame
{
    uint32_t id;             // Frame Identifier.
    can_frame_flags_t flags; // Frame flags, bitfield for eff(0x01), fd(0x02), brs(0x04), esi(0x08).
    uint8_t dlc;             // Data length code, past 8 bytes does not match the length of the frame itself. refer to dlc_map.
    uint8_t data[64];        // Frame payload data buffer.
} can_frame_t;

/**
 * @brief Decodes a frames Data Length Code into a useful byte length.
 * @param frame The CAN frame.
 * @return The data byte length of the frame provided.
 */
static inline uint8_t can_frame_get_length(const can_frame_t *frame)
{
    static const uint8_t dlc_map[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

    // Any invalid values just return 64 to be safe.
    if (frame->dlc > 15)
        return 64;
    return dlc_map[frame->dlc];
}

/**
 * @enum can_baudrates
 * @brief Enumeration of common CAN baud rates, including both standard CAN and CAN FD data rates.
 *        These baud rates can be used to configure the CAN controller for communication at the desired speed.
 *
 * @note The actual achievable baud rates may depend on the specific CAN controller hardware and the clock frequency used.
 */
typedef enum can_baudrates
{
    CAN_BAUD_10KBPS,
    CAN_BAUD_20KBPS,
    CAN_BAUD_50KBPS,
    CAN_BAUD_100KBPS,
    CAN_BAUD_125KBPS,
    CAN_BAUD_250KBPS,
    CAN_BAUD_500KBPS,
    CAN_BAUD_1MBPS,
    CAN_BAUD_2MBPS,
    CAN_BAUD_4MBPS,
    CAN_BAUD_8MBPS,
    CAN_BAUD_MAX
} can_baudrates_t;

#endif // __CAN_H__