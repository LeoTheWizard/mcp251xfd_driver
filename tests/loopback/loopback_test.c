/**
 * @file loopback_test.c
 * @brief MCP251xFD loopback self-test for RP2350 (Raspberry Pi Pico 2).
 *
 * Drives the lw_mcp251xfd driver in internal loopback mode and verifies that
 * every transmitted frame is received back intact. Results print over USB
 * serial at 115200 baud.
 *
 * Change MCP_* defines below to match your board wiring; set MCP_FOSC to the
 * crystal frequency on your MCP251xFD breakout.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/spi.h"
#include "mcp251xfd.h"

/* ---------- Hardware configuration ---------- */
#define MCP_SPI_PORT spi0
#define MCP_SPI_HZ 8000000u
#define MCP_PIN_SCK 2u
#define MCP_PIN_MOSI 3u
#define MCP_PIN_MISO 4u
#define MCP_PIN_CS 5u
#define MCP_FOSC MCP251XFD_FOSC_40MHZ
/* -------------------------------------------- */

#define TX_FIFO 1u
#define RX_FIFO 2u
#define RX_TIMEOUT_US 10000u /* 10 ms per frame — plenty for loopback */

/* ------------------------------------------------------------------ */
/* SPI platform callbacks — bridge the driver's HAL to pico-sdk SPI.    */
/* ------------------------------------------------------------------ */

typedef struct
{
    spi_inst_t *spi;
    uint cs_pin;
} spi_ctx_t;

/* CS is active-low: enable=true drives the pin low. */
static void chip_enable(void *iface, bool enable)
{
    spi_ctx_t *c = (spi_ctx_t *)iface;
    gpio_put(c->cs_pin, !enable);
}

/* Full-duplex SPI helper covering all three driver call patterns. */
static void spi_xfer(void *iface, const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_ctx_t *c = (spi_ctx_t *)iface;
    if (tx && rx)
        spi_write_read_blocking(c->spi, tx, rx, len);
    else if (tx)
        spi_write_blocking(c->spi, tx, len);
    else
        spi_read_blocking(c->spi, 0x00, rx, len);
}

static uint32_t get_us(void) { return time_us_32(); }
static void wait_us(uint32_t us) { sleep_us(us); }

/* ------------------------------------------------------------------ */
/* Minimal test runner                                                  */
/* ------------------------------------------------------------------ */

static int pass_count;
static int fail_count;

/* Print a single test result line and update pass/fail counters. */
static void report(const char *name, bool ok, const char *detail)
{
    printf("  %-52s %s", name, ok ? "PASS" : "FAIL");
    if (!ok && detail)
        printf(" (%s)", detail);
    printf("\n");
    if (ok)
        pass_count++;
    else
        fail_count++;
}

/* Send one frame, wait for it to arrive in the RX FIFO, then compare. */
static void run_test(MCP251XFD *dev, const char *name, const can_frame_t *tx)
{
    /* Flush any stale frames the previous test may have left behind. */
    mcp251xfd_flush_rx(dev, RX_FIFO);

    /* Queue the frame for transmission. */
    if (mcp251xfd_transmit(dev, TX_FIFO, tx) != MCP251XFD_RETURN_OK)
    {
        report(name, false, mcp251xfd_get_error_msg());
        return;
    }

    /* Poll until at least one frame is sitting in the RX FIFO. */
    uint32_t t0 = time_us_32();
    uint8_t pending = 0;
    while (!pending)
    {
        mcp251xfd_rx_pending(dev, RX_FIFO, &pending);
        if (time_us_32() - t0 > RX_TIMEOUT_US)
        {
            report(name, false, "rx timeout");
            return;
        }
    }

    /* Read it back and verify it matches what we sent. */
    can_frame_t rx;
    if (mcp251xfd_get_received(dev, RX_FIFO, &rx) != MCP251XFD_RETURN_OK)
    {
        report(name, false, "get_received failed");
        return;
    }

    if (rx.id != tx->id)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "id %08X != %08X",
                 (unsigned)rx.id, (unsigned)tx->id);
        report(name, false, buf);
        return;
    }
    if (rx.flags != tx->flags)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "flags %02X != %02X", rx.flags, tx->flags);
        report(name, false, buf);
        return;
    }
    if (rx.dlc != tx->dlc)
    {
        char buf[32];
        snprintf(buf, sizeof(buf), "dlc %u != %u", rx.dlc, tx->dlc);
        report(name, false, buf);
        return;
    }

    uint8_t len = can_frame_get_length(&rx);
    if (memcmp(rx.data, tx->data, len) != 0)
    {
        report(name, false, "data mismatch");
        return;
    }

    report(name, true, NULL);
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */

int main(void)
{
    stdio_init_all();
    sleep_ms(2000); /* wait for USB serial to enumerate */

    printf("\n=== MCP251xFD Loopback Test (RP2350) ===\n\n");

    /* Bring up the SPI peripheral and CS GPIO. */
    spi_init(MCP_SPI_PORT, MCP_SPI_HZ);
    gpio_set_function(MCP_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(MCP_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(MCP_PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(MCP_PIN_CS);
    gpio_set_dir(MCP_PIN_CS, GPIO_OUT);
    gpio_put(MCP_PIN_CS, 1); /* deassert */

    spi_ctx_t ctx = {MCP_SPI_PORT, MCP_PIN_CS};

    /* Allocate the driver instance. */
    MCP251XFD *dev = mcp251xfd_create_instance();
    if (!dev)
    {
        printf("FATAL: mcp251xfd_create_instance returned NULL\n");
        return 1;
    }

    /* One TX FIFO and one RX FIFO, both 64-byte payloads. Depth 16 on RX
     * keeps the total RAM budget under the chip's 2 KB limit. */
    mcp251xfd_fifo_config_t fifos[] = {
        {.tx = true, .depth = 8, .payload = MCP251XFD_PLSIZE_64, .tx_priority = 0, .auto_rtr = false},
        {.tx = false, .depth = 16, .payload = MCP251XFD_PLSIZE_64, .tx_priority = 0, .auto_rtr = false},
    };

    mcp251xfd_config_t cfg = {
        .elapsed_us = get_us,
        .delay_func = wait_us,
        .chip_enable = chip_enable,
        .spi_transfer = spi_xfer,
        .iface = &ctx,
        .fosc = MCP_FOSC,
        .model = MODEL_MCP2518FD,
        .nominal_baud = CAN_BAUD_500KBPS,
        .data_baud = CAN_BAUD_2MBPS,
        .enable_ecc = false,
        .fifo_configs = fifos,
        .fifo_count = 2,
    };

    /* Reset the chip, configure clocks, FIFOs and bit timing. */
    printf("Initialising driver...    ");
    if (mcp251xfd_initialise(dev, &cfg) != MCP251XFD_RETURN_OK)
    {
        printf("FAIL: %s\n", mcp251xfd_get_error_msg());
        reset_usb_boot(0, 0);
        return 1;
    }
    printf("OK\n");

    mcp251xfd_model_t model;
    if (mcp251xfd_get_model(dev, &model) == MCP251XFD_RETURN_OK)
        printf("Device identified as:               %s\n",
               model == MODEL_MCP2518FD ? "MCP2518FD" : "MCP2517FD");

    /* Two accept-all filters route every standard (filter 0) and extended
     * (filter 1) frame to RX_FIFO. Mask=0 means all ID bits are wildcards. */
    mcp251xfd_configure_filter(dev, 0, 0, 0, false, RX_FIFO);
    mcp251xfd_configure_filter(dev, 1, 0, 0, true, RX_FIFO);

    /* Switch to internal loopback: TX frames feed straight into RX. */
    printf("Entering internal loopback mode...  ");
    if (mcp251xfd_change_opmode(dev, MCP251XFD_OPMODE_INTERNAL_LOOPBACK, 10000) != MCP251XFD_RETURN_OK)
    {
        printf("FAIL\n");
        return 1;
    }
    printf("OK\n\n");

    /* -------- Test cases -------- */
    printf("Running tests:\n");

    /* 1. Classic CAN, 11-bit ID, 8 data bytes. */
    {
        can_frame_t f = {.id = 0x123, .flags = 0, .dlc = 8, .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}};
        run_test(dev, "Standard 11-bit ID, DLC=8", &f);
    }

    /* 2. Classic CAN, 29-bit extended ID. */
    {
        can_frame_t f = {.id = 0x1ABCDEF, .flags = CAN_FRAME_FLAG_EEF, .dlc = 8, .data = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE}};
        run_test(dev, "Extended 29-bit ID, DLC=8", &f);
    }

    /* 3. CAN FD with bit rate switch, max 64-byte payload. */
    {
        can_frame_t f;
        f.id = 0x456;
        f.flags = CAN_FRAME_FLAG_FDF | CAN_FRAME_FLAG_BRS;
        f.dlc = 15; /* DLC 15 = 64 bytes */
        for (int i = 0; i < 64; i++)
            f.data[i] = (uint8_t)(i ^ 0xA5);
        run_test(dev, "Standard FD + BRS, DLC=15 (64 bytes)", &f);
    }

    /* 4. CAN FD + BRS with a 29-bit extended ID and full 64-byte payload. */
    {
        can_frame_t f;
        f.id = 0x18FFAA55;
        f.flags = CAN_FRAME_FLAG_EEF | CAN_FRAME_FLAG_FDF | CAN_FRAME_FLAG_BRS;
        f.dlc = 15;
        for (int i = 0; i < 64; i++)
            f.data[i] = (uint8_t)(0xFF - i);
        run_test(dev, "Extended FD + BRS, DLC=15 (64 bytes)", &f);
    }

    /* 5. CAN FD without BRS, mid-sized 32-byte payload. */
    {
        can_frame_t f;
        f.id = 0x100;
        f.flags = CAN_FRAME_FLAG_FDF;
        f.dlc = 13; /* DLC 13 = 32 bytes */
        for (int i = 0; i < 32; i++)
            f.data[i] = (uint8_t)i;
        run_test(dev, "Standard FD, no BRS, DLC=13 (32 bytes)", &f);
    }

    /* 6. Zero-length classic frame with the largest standard ID. */
    {
        can_frame_t f = {.id = 0x7FF, .flags = 0, .dlc = 0};
        run_test(dev, "Standard 11-bit, DLC=0 (empty)", &f);
    }

    /* 7. Zero-length extended frame with the largest 29-bit ID. */
    {
        can_frame_t f = {.id = 0x1FFFFFFF, .flags = CAN_FRAME_FLAG_EEF, .dlc = 0};
        run_test(dev, "Extended 29-bit, DLC=0 (empty)", &f);
    }

    /* 8. All-ones payload at maximum standard ID — checks data integrity. */
    {
        can_frame_t f = {.id = 0x7FF, .flags = 0, .dlc = 8, .data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
        run_test(dev, "Standard max ID (0x7FF), all-0xFF data", &f);
    }

    /* 9. Queue 8 frames back-to-back without reading, then drain in order. */
    {
        /* Start with an empty RX FIFO so we read exactly what we queue. */
        mcp251xfd_flush_rx(dev, RX_FIFO);

        bool ok = true;

        /* Burst of 8 short frames, IDs 0x200..0x207, payload = sequence number. */
        for (uint8_t n = 0; n < 8 && ok; n++)
        {
            can_frame_t f = {.id = (uint32_t)(0x200 + n), .flags = 0, .dlc = 1, .data = {n}};
            if (mcp251xfd_transmit(dev, TX_FIFO, &f) != MCP251XFD_RETURN_OK)
            {
                ok = false;
                break;
            }
        }

        /* Read them back and check order + payload matches the burst. */
        if (ok)
        {
            for (uint8_t n = 0; n < 8 && ok; n++)
            {
                uint32_t t0 = time_us_32();
                uint8_t pending = 0;
                while (!pending)
                {
                    mcp251xfd_rx_pending(dev, RX_FIFO, &pending);
                    if (time_us_32() - t0 > RX_TIMEOUT_US)
                    {
                        ok = false;
                        break;
                    }
                }
                if (!ok)
                    break;
                can_frame_t rx;
                mcp251xfd_get_received(dev, RX_FIFO, &rx);
                if (rx.id != (uint32_t)(0x200 + n) || rx.data[0] != n)
                    ok = false;
            }
        }
        report("8x back-to-back standard frames", ok, ok ? NULL : "mismatch");
    }

    /* -------- Summary -------- */
    printf("\n=== %d passed, %d failed ===\n", pass_count, fail_count);

    mcp251xfd_destroy_instance(dev);

    /* Drop into the USB bootloader immediately on any failure for fast iteration. */
    if (fail_count != 0)
        reset_usb_boot(0, 0);

    /* On all-pass, blink GPIO 19 slowly for 10 s, then reboot to bootloader. */
    gpio_init(19);
    gpio_set_dir(19, GPIO_OUT);
    uint32_t deadline = time_us_32() + 10000000u;
    while ((int32_t)(deadline - time_us_32()) > 0)
    {
        gpio_put(19, 1);
        sleep_ms(500);
        gpio_put(19, 0);
        sleep_ms(500);
    }
    gpio_put(19, 0);
    reset_usb_boot(0, 0);
}
