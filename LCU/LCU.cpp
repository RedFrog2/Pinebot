#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "pico/stdlib.h"

#include "board_config.hpp"
#include "rov/can_protocol.hpp"
#include "rov/mcp2515.hpp"

/*
 * LCU -- LoRa Control Unit.
 *
 * Receives commands over LoRa and republishes them on the CAN bus for the RCU
 * to act on:
 *
 *      topside --LoRa--> [LCU] --CAN--> [RCU] --PWM--> ESC + servos
 *
 * The LoRa half is not implemented yet (the module is not chosen), so the
 * command source is currently the USB serial console. That is deliberately
 * useful rather than a placeholder: it lets the whole CAN path and the RCU be
 * brought up and tested end to end before any radio exists. When the module
 * arrives, only feed_from_lora() has to be written -- it sets the same
 * `command` struct and calls note_command(), and nothing else changes.
 *
 * Serial commands (one per line, 115200 baud, rate ignored by USB CDC):
 *
 *      <number>    thruster setpoint, -1.0 .. +1.0
 *      sN <num>    servo N (0..3) position, -1.0 .. +1.0
 *      s | stop    everything to neutral
 *      can         MCP2515 bring-up: loopback self test + bus health
 *      ?           print current state
 *
 * FAIL-SILENT: if no command arrives for kSourceTimeoutMs, this board STOPS
 * PUBLISHING rather than repeating the last value. That is the contract in
 * can_protocol.hpp -- the RCU cannot distinguish stale data from fresh, so
 * silence is the only honest way to report a dead uplink, and it lets the
 * RCU's own timeout neutral the motor.
 */

namespace {

using rov::can::Command;
using rov::can::Status;

constexpr uint32_t kSourceTimeoutMs = 500;
constexpr size_t   kLineMax = 48;

Mcp2515 can;
bool    can_ok = false;

Command command;
bool    source_live = false;
absolute_time_t last_source;

char   line[kLineMax];
size_t line_len = 0;

uint32_t frames_sent = 0;
Status   last_status;
bool     have_status = false;

void led_init()
{
    gpio_init(board::kLedGpio);
    gpio_set_dir(board::kLedGpio, GPIO_OUT);
}

/* Mark the command source alive. Called by the serial console today and by
 * the LoRa receiver once it exists. */
void note_command()
{
    last_source = get_absolute_time();
    source_live = true;
}

void stop_all()
{
    command.thruster = 0.0f;
    for (int i = 0; i < rov::can::kServoCount; i++) {
        command.servo[i] = 0.0f;
    }
    command.flags = 0;
}

void print_state()
{
    printf("state t=%.3f s=[%.2f %.2f %.2f %.2f] seq=%u live=%d sent=%lu can=%d\n",
           static_cast<double>(command.thruster),
           static_cast<double>(command.servo[0]), static_cast<double>(command.servo[1]),
           static_cast<double>(command.servo[2]), static_cast<double>(command.servo[3]),
           command.seq, source_live ? 1 : 0,
           static_cast<unsigned long>(frames_sent), can_ok ? 1 : 0);
    if (have_status) {
        printf("rcu   t=%.3f us=%u flags=0x%02X ack_seq=%u eflg=0x%02X tec=%u\n",
               static_cast<double>(last_status.thruster), last_status.pulse_us,
               last_status.flags, last_status.last_seq,
               last_status.can_eflg, last_status.can_tec);
    } else {
        printf("rcu   no status frames received\n");
    }
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

/* Parse "sN <value>". Returns false if it is not a servo command at all. */
bool handle_servo(const char *cmd)
{
    if (cmd[0] != 's' || cmd[1] < '0' || cmd[1] > '3') {
        return false;
    }
    const int index = cmd[1] - '0';

    char *end = nullptr;
    float value = strtof(cmd + 2, &end);
    if (end == cmd + 2 || *end != '\0') {
        printf("err bad servo value '%s'\n", cmd);
        return true;
    }
    if (value < -1.0f || value > 1.0f) {
        printf("err out of range %.3f\n", static_cast<double>(value));
        return true;
    }

    command.servo[index] = value;
    note_command();
    printf("ok servo%d=%.3f\n", index, static_cast<double>(value));
    return true;
}

void handle_line(char *cmd)
{
    size_t n = strlen(cmd);
    while (n > 0 && (cmd[n - 1] == ' ' || cmd[n - 1] == '\t' || cmd[n - 1] == '\r')) {
        cmd[--n] = '\0';
    }
    if (n == 0) {
        return;
    }

    if (strcmp(cmd, "?") == 0) {
        print_state();
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
    if (strcmp(cmd, "s") == 0 || strcmp(cmd, "stop") == 0) {
        stop_all();
        note_command();
        printf("ok stop\n");
        return;
    }
    if (handle_servo(cmd)) {
        return;
    }

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

    command.thruster = value;
    command.flags    = rov::can::kFlagEnable;
    note_command();
    printf("ok t=%.3f\n", static_cast<double>(value));
}

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
            line_len = 0;
            printf("err line too long\n");
        }
    }
}

/* TODO: called once the LoRa module is chosen. It must only call
 * note_command() when a frame genuinely arrived -- never on a timer, or the
 * fail-silent property is lost and a dead radio looks alive to the RCU. */
void poll_lora()
{
}

void poll_source_timeout()
{
    if (!source_live) {
        return;
    }
    if (absolute_time_diff_us(last_source, get_absolute_time()) >
        static_cast<int64_t>(kSourceTimeoutMs) * 1000) {
        source_live = false;
        stop_all();
        printf("err uplink quiet for %u ms, ceasing to publish\n", kSourceTimeoutMs);
    }
}

/* Publish only while the uplink is alive. Silence is the signal. */
void publish_command()
{
    if (!can_ok || !source_live) {
        return;
    }

    uint8_t payload[rov::can::kDlc];
    command.seq++;
    rov::can::encode(command, payload);

    Mcp2515::Frame frame{};
    frame.id  = rov::can::kIdCommand;
    frame.len = rov::can::kDlc;
    memcpy(frame.data, payload, rov::can::kDlc);

    if (can.send(frame)) {
        frames_sent++;
    }
}

void poll_status()
{
    if (!can_ok) {
        return;
    }
    Mcp2515::Frame frame;
    while (can.receive(&frame)) {
        if (frame.id == rov::can::kIdStatus && frame.len == rov::can::kDlc) {
            rov::can::decode(frame.data, &last_status);
            have_status = true;
        }
    }
}

}  // namespace

int main()
{
    stdio_init_all();
    led_init();

    can_ok = can.init(make_can_config());

    absolute_time_t next_publish = get_absolute_time();
    absolute_time_t next_blink   = get_absolute_time();

    while (true) {
        poll_serial();
        poll_lora();
        poll_source_timeout();
        poll_status();

        if (absolute_time_diff_us(next_publish, get_absolute_time()) > 0) {
            publish_command();
            next_publish = delayed_by_ms(get_absolute_time(),
                                         1000 / rov::can::kCommandRateHz);
        }

        /* Fast blink while publishing, slow while the uplink is quiet. */
        if (absolute_time_diff_us(next_blink, get_absolute_time()) > 0) {
            gpio_xor_mask(1u << board::kLedGpio);
            next_blink = delayed_by_ms(get_absolute_time(), source_live ? 100 : 500);
        }

        sleep_ms(1);
    }
}
