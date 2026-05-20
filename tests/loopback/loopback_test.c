/**
 * @file loopback_test.c
 * @brief MCP251xFD loopback self-test for RP2350 (Raspberry Pi Pico 2).
 *
 * Exercises the lw_mcp251xfd driver by placing the MCP251xFD in internal
 * loopback mode and verifying that every transmitted frame is received back
 * intact.  Results are printed over USB serial at 115200 baud.
 *
 * Default wiring (SPI0):
 *   GPIO 2  — SCK
 *   GPIO 3  — MOSI (TX)
 *   GPIO 4  — MISO (RX)
 *   GPIO 5  — CS   (active-low, driven by this code)
 *
 * Change the MCP_* defines below if your board is wired differently.
 * Set MCP_FOSC to match the crystal on your MCP251xFD breakout board.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "mcp251xfd.h"

/* ---------- Hardware configuration ---------- */
#define MCP_SPI_PORT spi0
#define MCP_SPI_HZ 4000000u /* 4 MHz SPI clock                  */
#define MCP_PIN_SCK 2u
#define MCP_PIN_MOSI 3u
#define MCP_PIN_MISO 4u
#define MCP_PIN_CS 5u
#define MCP_FOSC MCP251XFD_FOSC_40MHZ /* crystal on the breakout board */
/* -------------------------------------------- */

#define TX_FIFO 1u
#define RX_FIFO 2u
#define RX_TIMEOUT_US 10000u /* 10 ms per frame — plenty for loopback     */

/* ------------------------------------------------------------------ */
/* SPI platform callbacks                                               */
/* ------------------------------------------------------------------ */

typedef struct
{
    spi_inst_t *spi;
    uint cs_pin;
} spi_ctx_t;

static void chip_enable(void *iface, bool enable)
{
    spi_ctx_t *c = (spi_ctx_t *)iface;
    gpio_put(c->cs_pin, !enable); /* CS is active-low */
}

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

static void run_test(MCP251XFD *dev, const char *name, const can_frame_t *tx)
{
    if (mcp251xfd_transmit(dev, TX_FIFO, tx) != MCP251XFD_RETURN_OK)
    {
        report(name, false, "transmit failed");
        return;
    }

    /* Poll for the frame to appear in the RX FIFO. */
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

    /* Initialise SPI0. */
    spi_init(MCP_SPI_PORT, MCP_SPI_HZ);
    spi_set_format(MCP_SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(MCP_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(MCP_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(MCP_PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(MCP_PIN_CS);
    gpio_set_dir(MCP_PIN_CS, GPIO_OUT);
    gpio_put(MCP_PIN_CS, 1); /* deassert */

    spi_ctx_t ctx = {MCP_SPI_PORT, MCP_PIN_CS};

    /* Allocate and initialise the driver. */
    MCP251XFD *dev = mcp251xfd_create_instance();
    if (!dev)
    {
        printf("FATAL: mcp251xfd_create_instance returned NULL\n");
        return 1;
    }

    mcp251xfd_fifo_config_t fifos[] = {
        /* FIFO 1 — TX: depth 8, 64-byte payload */
        {.tx = true, .depth = 8, .payload = MCP251XFD_PLSIZE_64, .tx_priority = 0, .auto_rtr = false},
        /* FIFO 2 — RX: depth 32, 64-byte payload */
        {.tx = false, .depth = 32, .payload = MCP251XFD_PLSIZE_64, .tx_priority = 0, .auto_rtr = false},
    };

    mcp251xfd_config_t cfg = {
        .elapsed_us = get_us,
        .delay_func = wait_us,
        .chip_enable = chip_enable,
        .spi_transfer = spi_xfer,
        .iface = &ctx,
        .fosc = MCP_FOSC,
        .nominal_baud = CAN_BAUD_500KBPS,
        .data_baud = CAN_BAUD_2MBPS,
        .enable_ecc = false,
        .fifo_configs = fifos,
        .fifo_count = 2,
    };

    printf("Initialising driver...              ");
    if (mcp251xfd_initialise(dev, &cfg) != MCP251XFD_RETURN_OK)
    {
        printf("FAIL: %s\n", mcp251xfd_get_error_msg());
        return 1;
    }
    printf("OK\n");

    mcp251xfd_model_t model;
    if (mcp251xfd_get_model(dev, &model) == MCP251XFD_RETURN_OK)
        printf("Device identified as:               %s\n",
               model == MODEL_MCP2518FD ? "MCP2518FD" : "MCP2517FD");

    /*
     * Accept all standard frames on filter 0, all extended frames on filter 1,
     * both routed to RX_FIFO.  Mask = 0 means all ID bits are don't-care.
     */
    mcp251xfd_configure_filter(dev, 0, 0, 0, false, RX_FIFO);
    mcp251xfd_configure_filter(dev, 1, 0, 0, true, RX_FIFO);

    printf("Entering internal loopback mode...  ");
    if (mcp251xfd_change_opmode(dev, MCP251XFD_OPMODE_INTERNAL_LOOPBACK, 10000) != MCP251XFD_RETURN_OK)
    {
        printf("FAIL\n");
        return 1;
    }
    printf("OK\n\n");

    /* -------- Test cases -------- */
    printf("Running tests:\n");

    /* 1. Standard CAN frame, 8 bytes */
    {
        can_frame_t f = {.id = 0x123, .flags = 0, .dlc = 8, .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}};
        run_test(dev, "Standard 11-bit ID, DLC=8", &f);
    }

    /* 2. Extended CAN frame, 8 bytes */
    {
        can_frame_t f = {.id = 0x1ABCDEF, .flags = CAN_FRAME_FLAG_EEF, .dlc = 8, .data = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE}};
        run_test(dev, "Extended 29-bit ID, DLC=8", &f);
    }

    /* 3. CAN FD standard frame, 64 bytes */
    {
        can_frame_t f;
        f.id = 0x456;
        f.flags = CAN_FRAME_FLAG_FDF | CAN_FRAME_FLAG_BRS;
        f.dlc = 15; /* DLC 15 = 64 bytes */
        for (int i = 0; i < 64; i++)
            f.data[i] = (uint8_t)(i ^ 0xA5);
        run_test(dev, "Standard FD + BRS, DLC=15 (64 bytes)", &f);
    }

    /* 4. CAN FD extended frame, 64 bytes */
    {
        can_frame_t f;
        f.id = 0x18FFAA55;
        f.flags = CAN_FRAME_FLAG_EEF | CAN_FRAME_FLAG_FDF | CAN_FRAME_FLAG_BRS;
        f.dlc = 15;
        for (int i = 0; i < 64; i++)
            f.data[i] = (uint8_t)(0xFF - i);
        run_test(dev, "Extended FD + BRS, DLC=15 (64 bytes)", &f);
    }

    /* 5. CAN FD standard frame, no BRS, 32 bytes */
    {
        can_frame_t f;
        f.id = 0x100;
        f.flags = CAN_FRAME_FLAG_FDF;
        f.dlc = 13; /* DLC 13 = 32 bytes */
        for (int i = 0; i < 32; i++)
            f.data[i] = (uint8_t)i;
        run_test(dev, "Standard FD, no BRS, DLC=13 (32 bytes)", &f);
    }

    /* 6. Standard CAN, DLC=0 (zero-length data frame) */
    {
        can_frame_t f = {.id = 0x7FF, .flags = 0, .dlc = 0};
        run_test(dev, "Standard 11-bit, DLC=0 (empty)", &f);
    }

    /* 7. Extended CAN, DLC=0 */
    {
        can_frame_t f = {.id = 0x1FFFFFFF, .flags = CAN_FRAME_FLAG_EEF, .dlc = 0};
        run_test(dev, "Extended 29-bit, DLC=0 (empty)", &f);
    }

    /* 8. Standard CAN, max ID, all-ones data */
    {
        can_frame_t f = {.id = 0x7FF, .flags = 0, .dlc = 8, .data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};
        run_test(dev, "Standard max ID (0x7FF), all-0xFF data", &f);
    }

    /* 9. Multiple back-to-back standard frames */
    {
        bool ok = true;
        for (uint8_t n = 0; n < 8 && ok; n++)
        {
            can_frame_t f = {.id = (uint32_t)(0x200 + n), .flags = 0, .dlc = 1, .data = {n}};
            if (mcp251xfd_transmit(dev, TX_FIFO, &f) != MCP251XFD_RETURN_OK)
            {
                ok = false;
                break;
            }
        }
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

#ifdef PICO_DEFAULT_LED_PIN
    /* Slow blink = all pass; fast blink = failures. */
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    while (true)
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(fail_count == 0 ? 500 : 100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(fail_count == 0 ? 500 : 100);
    }
#else
    while (true)
        sleep_ms(1000);
#endif
}
