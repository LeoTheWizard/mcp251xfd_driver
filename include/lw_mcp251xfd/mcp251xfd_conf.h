/**
 * @file mcp2518fd_conf.h
 * @brief Configuration definitions and constants for the MCP251xFD CAN controller device driver.
 *
 * @author Leo Walker
 *
 *  THIS IS A TEMPLATE CONFIGURATION HEADER FILE FOR THE MCP251xFD DRIVER.
 *  Users should copy this file to their project and modify the configuration parameters as needed for their specific application and hardware setup.
 */

#ifndef MCP2518FD_CONF_H
#define MCP2518FD_CONF_H

/**
 * @brief Define this macro to enable NULL parameter checking in the driver functions.
 */
#define MCP251XFD_CHECK_NULL_PARAM

/**
 * @brief Define this macro to enable plain text error messages that can be retrieved with mcp251xfd_get_error_msg().
 * A 128-byte buffer is allocated for storing error messages.
 */
#define MCP251XFD_ENABLE_ERROR_MESSAGES

/**
 * @brief Number of times a CRC-protected read is retried before the sticky CRC
 * error flag is latched. Only relevant when a device is initialised with use_crc.
 */
#ifndef MCP251XFD_CRC_RETRIES
#define MCP251XFD_CRC_RETRIES 3
#endif

#endif /* MCP2518FD_CONF_H */