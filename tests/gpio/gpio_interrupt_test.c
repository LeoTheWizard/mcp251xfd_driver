#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/spi.h"
#include <lw_mcp251xfd/mcp251xfd.h>

/* ---------- Hardware configuration ---------- */
// #define MCP_SPI_PORT spi1
// #define MCP_SPI_HZ 8000000u
// #define MCP_PIN_SCK 14u
// #define MCP_PIN_MOSI 15u
// #define MCP_PIN_MISO 12u
// #define MCP_PIN_CS 13u
// #define MCP_FOSC MCP251XFD_FOSC_40MHZ
/* -------------------------------------------- */

/* ---------- Test Board configuration ---------- */
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
#define RX_TIMEOUT_US 10000u

typedef struct
{
    spi_inst_t *spi;
    uint cs_pin;
} spi_ctx_t;

static void chip_enable(void *iface, bool enable)
{
    spi_ctx_t *c = (spi_ctx_t *)iface;
    gpio_put(c->cs_pin, !enable);
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

static int pass_count;
static int fail_count;

static void report(const char *name, bool ok, const char *detail)
{
    printf("  %-44s %s", name, ok ? "PASS" : "FAIL");
    if (!ok && detail)
        printf(" (%s)", detail);
    printf("\n");
    if (ok)
        pass_count++;
    else
        fail_count++;
}

static bool expect_ok(const char *name, mcp251xfd_return_t rc)
{
    bool ok = rc == MCP251XFD_RETURN_OK;
    report(name, ok, ok ? NULL : mcp251xfd_get_error_msg());
    return ok;
}

static bool expect_bool(const char *name, bool actual, bool expected)
{
    char buf[32];
    bool ok = actual == expected;
    if (!ok)
    {
        snprintf(buf, sizeof(buf), "%u != %u", (unsigned)actual, (unsigned)expected);
    }
    report(name, ok, ok ? NULL : buf);
    return ok;
}

static bool wait_for_interrupt_flags(MCP251XFD *dev, uint32_t mask, uint32_t timeout_us, uint32_t *out_flags)
{
    uint32_t start = time_us_32();
    while (time_us_32() - start < timeout_us)
    {
        uint32_t flags = 0;
        if (mcp251xfd_get_interrupt_flags(dev, &flags) != MCP251XFD_RETURN_OK)
            return false;
        if ((flags & mask) != 0)
        {
            if (out_flags)
                *out_flags = flags;
            return true;
        }
    }
    return false;
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== MCP251xFD GPIO + Interrupt Test ===\n\n");

    spi_init(MCP_SPI_PORT, MCP_SPI_HZ);
    gpio_set_function(MCP_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(MCP_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(MCP_PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(MCP_PIN_CS);
    gpio_set_dir(MCP_PIN_CS, GPIO_OUT);
    gpio_put(MCP_PIN_CS, 1);

    spi_ctx_t ctx = {MCP_SPI_PORT, MCP_PIN_CS};

    MCP251XFD *dev = mcp251xfd_create_instance();
    if (!dev)
    {
        printf("FATAL: mcp251xfd_create_instance returned NULL\n");
        return 1;
    }

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

    printf("Initialising driver...    ");
    if (mcp251xfd_initialise(dev, &cfg) != MCP251XFD_RETURN_OK)
    {
        printf("FAIL: %s\n", mcp251xfd_get_error_msg());
        reset_usb_boot(0, 0);
        return 1;
    }
    printf("OK\n");

    mcp251xfd_configure_filter(dev, 0, 0, 0, false, RX_FIFO);
    mcp251xfd_configure_filter(dev, 1, 0, 0, true, RX_FIFO);

    printf("Entering internal loopback mode...  ");
    if (mcp251xfd_change_opmode(dev, MCP251XFD_OPMODE_INTERNAL_LOOPBACK, 10000) != MCP251XFD_RETURN_OK)
    {
        printf("FAIL\n");
        return 1;
    }
    printf("OK\n\n");

    printf("Running GPIO + interrupt checks:\n");

    bool ok;
    bool all_ok = true;

    ok = expect_ok("GPIO0 -> GPIO mode", mcp251xfd_gpio_set_mode(dev, MCP251XFD_GPIO0, MCP251XFD_GPIO_MODE_GPIO));
    all_ok &= ok;
    ok = expect_ok("GPIO0 direction output", mcp251xfd_gpio_set_direction(dev, MCP251XFD_GPIO0, MCP251XFD_GPIO_OUTPUT));
    all_ok &= ok;
    ok = expect_ok("GPIO0 write high", mcp251xfd_gpio_write(dev, MCP251XFD_GPIO0, true));
    all_ok &= ok;
    if (ok)
    {
        bool value = false;
        if (mcp251xfd_gpio_read(dev, MCP251XFD_GPIO0, &value) == MCP251XFD_RETURN_OK)
            ok = expect_bool("GPIO0 read back high", value, true);
        else
            ok = false;
        all_ok &= ok;
    }

    ok = expect_ok("GPIO0 write low", mcp251xfd_gpio_write(dev, MCP251XFD_GPIO0, false));
    all_ok &= ok;
    if (ok)
    {
        bool value = true;
        if (mcp251xfd_gpio_read(dev, MCP251XFD_GPIO0, &value) == MCP251XFD_RETURN_OK)
            ok = expect_bool("GPIO0 read back low", value, false);
        else
            ok = false;
        all_ok &= ok;
    }

    ok = expect_ok("GPIO1 -> alternate mode", mcp251xfd_gpio_set_mode(dev, MCP251XFD_GPIO1, MCP251XFD_GPIO_MODE_ALT));
    all_ok &= ok;
    ok = expect_ok("GPIO1 disable SOF (CLKO mode)", mcp251xfd_gpio_set_sof_output(dev, false));
    all_ok &= ok;
    ok = expect_ok("GPIO1 enable SOF output", mcp251xfd_gpio_set_sof_output(dev, true));
    all_ok &= ok;
    ok = expect_ok("GPIO1 disable SOF output", mcp251xfd_gpio_set_sof_output(dev, false));
    all_ok &= ok;

    ok = expect_ok("INT open-drain enable", mcp251xfd_gpio_set_int_open_drain(dev, true));
    all_ok &= ok;
    ok = expect_ok("INT open-drain disable", mcp251xfd_gpio_set_int_open_drain(dev, false));
    all_ok &= ok;
    ok = expect_ok("TXCAN open-drain enable", mcp251xfd_gpio_set_tx_open_drain(dev, true));
    all_ok &= ok;
    ok = expect_ok("TXCAN open-drain disable", mcp251xfd_gpio_set_tx_open_drain(dev, false));
    all_ok &= ok;

    ok = expect_ok("Enable TX/RX interrupts", mcp251xfd_configure_interrupts(dev, MCP251XFD_INT_TX | MCP251XFD_INT_RX));
    all_ok &= ok;

    {
        uint32_t flags = 0;
        if (mcp251xfd_get_interrupt_flags(dev, &flags) == MCP251XFD_RETURN_OK)
        {
            report("Initial interrupt flags read", true, NULL);
        }
        else
        {
            report("Initial interrupt flags read", false, NULL);
            all_ok = false;
        }
    }

    {
        can_frame_t frame = {.id = 0x321, .flags = 0, .dlc = 2, .data = {0xAA, 0x55}};
        mcp251xfd_flush_rx(dev, RX_FIFO);
        if (mcp251xfd_transmit(dev, TX_FIFO, &frame) != MCP251XFD_RETURN_OK)
        {
            report("Transmit frame for interrupt test", false, mcp251xfd_get_error_msg());
            all_ok = false;
        }
        else
        {
            uint32_t flags = 0;
            if (wait_for_interrupt_flags(dev, MCP251XFD_INT_TX | MCP251XFD_INT_RX, RX_TIMEOUT_US, &flags))
            {
                report("TX/RX interrupt flags set", true, NULL);
                ok = expect_bool("TX or RX flag present", (flags & (MCP251XFD_INT_TX | MCP251XFD_INT_RX)) != 0, true);
                all_ok &= ok;
            }
            else
            {
                report("TX/RX interrupt flags set", false, "timeout");
                all_ok = false;
            }
        }
    }

    // RXIF is a read-only summary flag; it only clears once the RX FIFO is drained,
    // not by writing CiINT. Read out every pending frame, then confirm RXIF dropped.
    uint8_t pending = 0;
    mcp251xfd_rx_pending(dev, RX_FIFO, &pending);
    while (pending)
    {
        can_frame_t rx;
        if (mcp251xfd_get_received(dev, RX_FIFO, &rx) != MCP251XFD_RETURN_OK)
            break;
        mcp251xfd_rx_pending(dev, RX_FIFO, &pending);
    }

    {
        uint32_t flags = 0;
        if (mcp251xfd_get_interrupt_flags(dev, &flags) == MCP251XFD_RETURN_OK)
            ok = expect_bool("RX interrupt flag cleared after drain", (flags & MCP251XFD_INT_RX) != 0, false);
        else
            ok = false;
        all_ok &= ok;
    }

    printf("\nSummary: %d pass, %d fail\n", pass_count, fail_count);
    if (!all_ok)
        printf("GPIO + interrupt test completed with failures.\n");
    else
        printf("GPIO + interrupt test completed successfully.\n");

    while (true)
        tight_loop_contents();

    return fail_count ? 1 : 0;
}
