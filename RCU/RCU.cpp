#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "pico/stdlib.h"

#include "board_config.hpp"
#include "esc.hpp"
#include "rov/mcp2515.hpp"

/*
 * Thruster control over USB serial.
 *
 * The PC sends one command per line at 115200 baud (the rate is ignored by
 * USB CDC, any setting works). Commands:
 *
 *      <number>    throttle setpoint, -1.0 .. +1.0   e.g.  0.35
 *      s | stop    setpoint 0, motor neutral
 *      arm         re-run the ESC arming sequence (blocks ~2 s)
 *      fs on|off   enable/disable the comms failsafe
 *      can         MCP2515 bring-up: loopback self test + bus health
 *      ?           print current state
 *
 * Every command is acknowledged with a line starting "ok" or "err", so the
 * sender can tell the difference between a dropped link and a rejected value.
 *
 * FAILSAFE: with fs on (the default) the motor returns to neutral if no
 * command arrives for kFailsafeMs. A tether that gets yanked mid-throttle
 * stops the thruster instead of leaving it running. Senders are expected to
 * repeat the current setpoint as a keepalive; tools/send_speed.py does this at
 * 10 Hz. Turn it off with "fs off" when poking at it by hand from a terminal.
 */

namespace {

/* Neutral the motor if the link goes quiet for this long. */
constexpr uint32_t kFailsafeMs = 1000;

constexpr size_t kLineMax = 32;

Esc     thruster;
Mcp2515 can;
bool    can_ok = false;

char   line[kLineMax];
size_t line_len = 0;

float           setpoint = 0.0f;
bool            failsafe_enabled = true;
bool            failsafe_tripped = false;
absolute_time_t last_command;

void led_init()
{
    gpio_init(board::kLedGpio);
    gpio_set_dir(board::kLedGpio, GPIO_OUT);
}

/* A failed init leaves the motor unarmed and reports on the only output the
 * board is guaranteed to have. */
void panic_blink()
{
    while (true) {
        gpio_put(board::kLedGpio, 1);
        sleep_ms(80);
        gpio_put(board::kLedGpio, 0);
        sleep_ms(80);
    }
}

void apply(float throttle)
{
    setpoint = throttle;
    thruster.set_throttle(throttle);
}

void print_state()
{
    printf("state t=%.3f us=%u armed=%d fs=%s%s\n",
           static_cast<double>(setpoint),
           thruster.pulse_us(),
           thruster.is_armed() ? 1 : 0,
           failsafe_enabled ? "on" : "off",
           failsafe_tripped ? " TRIPPED" : "");
}

Mcp2515::Config make_can_config()
{
    Mcp2515::Config cfg;
    cfg.sck     = board::kCanSck;
    cfg.tx      = board::kCanTx;
    cfg.rx      = board::kCanRx;
    cfg.cs      = board::kCanCs;
    cfg.reset   = board::kCanReset;
    cfg.intr    = board::kCanInt;
    cfg.xtal_hz = board::kCanXtalHz;
    cfg.bitrate = board::kCanBitrate;
    return cfg;
}


/* Jumper test: with TX linked to RX at the Pico, this must echo. */
void report_spi_echo()
{
    const uint8_t tx[4] = {0xA5, 0x5A, 0xFF, 0x00};
    uint8_t rx[4] = {};
    can.bus_echo(make_can_config(), tx, rx, sizeof(tx));

    printf("spi echo: sent %02X %02X %02X %02X got %02X %02X %02X %02X\n",
           tx[0], tx[1], tx[2], tx[3], rx[0], rx[1], rx[2], rx[3]);
    if (memcmp(tx, rx, sizeof(tx)) == 0) {
        printf("ok spi echo: RP2040 SPI and pin map verified -- fault is "
               "beyond the Pico pins\n");
    } else {
        printf("err spi echo: no echo. With the jumper fitted this means the "
               "SPI pin map is wrong; with no jumper this is expected\n");
    }
}

/* Full MCP2515 bring-up report: wiring probe first (works even when init
 * failed), then re-init, then the loopback self test. */
void report_can()
{
    Mcp2515::Config cfg = make_can_config();

    const Mcp2515::SpiCheck check = can.check_wiring(cfg);
    printf("can wiring: cs=%s reset=%s\n",
           check.cs_pin_high ? "high" : "STUCK LOW",
           check.reset_pin_high ? "high" : "STUCK LOW");
    printf("can wiring: canstat=0x%02X cfgmode=%d int=%s readback=%d "
           "[%02X->%02X %02X->%02X %02X->%02X %02X->%02X]\n",
           check.canstat, check.in_config_mode ? 1 : 0,
           check.int_high ? "high" : "LOW", check.readback_ok ? 1 : 0,
           check.sent[0], check.got[0], check.sent[1], check.got[1],
           check.sent[2], check.got[2], check.sent[3], check.got[3]);
    if (!check.readback_ok) {
        printf("can wiring: SO line is %s\n",
               check.rx_follows_pulls ? "floating (no driver)"
               : check.rx_driven_low  ? "driven LOW"
               : check.rx_driven_high ? "driven HIGH"
                                      : "indeterminate");
    }
    printf("can wiring: %s\n", Mcp2515::diagnose(check));

    /* The probe leaves the chip reset, so bring it back up. */
    can_ok = can.init(cfg);
    if (!can_ok) {
        printf("err can: init failed after probe (bit timing or no response)\n");
        return;
    }

    const bool passed = can.self_test();
    printf("ok can selftest=%s mode=0x%02X tec=%u rec=%u eflg=0x%02X\n",
           passed ? "pass" : "FAIL",
           static_cast<unsigned>(can.mode()),
           can.tx_error_count(), can.rx_error_count(), can.error_flags());
}

void handle_line(char *cmd)
{
    /* Strip trailing whitespace so a stray \r from a terminal is harmless. */
    size_t n = strlen(cmd);
    while (n > 0 && (cmd[n - 1] == ' ' || cmd[n - 1] == '\t' || cmd[n - 1] == '\r')) {
        cmd[--n] = '\0';
    }
    if (n == 0) {
        return;
    }

    last_command     = get_absolute_time();
    failsafe_tripped = false;

    if (strcmp(cmd, "?") == 0) {
        print_state();
        return;
    }
    if (strcmp(cmd, "s") == 0 || strcmp(cmd, "stop") == 0) {
        apply(0.0f);
        printf("ok stop us=%u\n", thruster.pulse_us());
        return;
    }
    if (strcmp(cmd, "arm") == 0) {
        apply(0.0f);
        thruster.arm();
        printf("ok armed\n");
        return;
    }
    if (strcmp(cmd, "can") == 0) {
        report_can();
        return;
    }
    if (strcmp(cmd, "spiloop") == 0) {
        report_spi_echo();
        return;
    }
    if (strcmp(cmd, "fs on") == 0) {
        failsafe_enabled = true;
        printf("ok fs on\n");
        return;
    }
    if (strcmp(cmd, "fs off") == 0) {
        failsafe_enabled = false;
        printf("ok fs off\n");
        return;
    }

    /* Anything else must be a bare throttle value. strtof tells us whether it
     * consumed the whole token, which rejects junk like "0.5x". */
    char *end = nullptr;
    float value = strtof(cmd, &end);
    if (end == cmd || *end != '\0') {
        printf("err bad command '%s'\n", cmd);
        return;
    }
    if (value < -1.0f || value > 1.0f) {
        printf("err out of range %.3f\n", static_cast<double>(value));
        return;
    }

    apply(value);
    printf("ok t=%.3f us=%u\n", static_cast<double>(setpoint), thruster.pulse_us());
}

/* Drain whatever the host has sent without ever blocking, so the failsafe
 * keeps being evaluated even when the link is silent. */
void poll_serial()
{
    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            line[line_len] = '\0';
            handle_line(line);
            line_len = 0;
        } else if (line_len < kLineMax - 1) {
            line[line_len++] = static_cast<char>(c);
        } else {
            /* Overlong line: drop it rather than acting on a truncated value. */
            line_len = 0;
            printf("err line too long\n");
        }
    }
}

void poll_failsafe()
{
    if (!failsafe_enabled || failsafe_tripped) {
        return;
    }
    if (absolute_time_diff_us(last_command, get_absolute_time()) > kFailsafeMs * 1000) {
        failsafe_tripped = true;
        apply(0.0f);
        printf("err failsafe: no command for %u ms, motor neutral\n", kFailsafeMs);
    }
}

}  // namespace

int main()
{
    stdio_init_all();
    led_init();

    if (!thruster.init(Esc::Config(board::kEscGpio))) {
        panic_blink();
    }

    /* Holds neutral while the ESC runs its self test; it beeps when ready. */
    thruster.arm();

    /* CAN is not required to drive the thruster, so a failure here is
     * reported by the "can" command rather than being fatal. */
    can_ok = can.init(make_can_config());

    last_command = get_absolute_time();

    absolute_time_t next_blink = get_absolute_time();
    while (true) {
        poll_serial();
        poll_failsafe();

        if (absolute_time_diff_us(next_blink, get_absolute_time()) > 0) {
            gpio_xor_mask(1u << board::kLedGpio);
            next_blink = delayed_by_ms(get_absolute_time(), 500);
        }

        sleep_ms(1);
    }
}
