#ifndef MCP2515_HPP
#define MCP2515_HPP

#include <cstdint>

#include "hardware/spi.h"
#include "pico/stdlib.h"

/*
 * Driver for the MCP2515 stand-alone CAN controller on SPI.
 *
 * The MCP2515 does the CAN protocol itself; the RP2040 only talks to it over
 * SPI, so no PIO is involved. Bit timing comes from the MCP2515's OWN crystal,
 * not the RP2040 clock -- get Config::xtal_hz wrong and the controller sits on
 * the bus at the wrong rate, which looks exactly like a wiring fault.
 *
 * Usage:
 *
 *      Mcp2515 can;
 *      Mcp2515::Config cfg;            // override pins from board_config.hpp
 *      if (!can.init(cfg)) { ... }     // false = no working SPI link
 *
 *      Mcp2515::Frame tx{};
 *      tx.id = 0x123;
 *      tx.len = 2;
 *      tx.data[0] = 0xDE; tx.data[1] = 0xAD;
 *      can.send(tx);
 *
 *      Mcp2515::Frame rx;
 *      if (can.receive(&rx)) { ... }
 *
 * Before trusting the bus, call self_test(): it loops a frame back inside the
 * chip with the transceiver disconnected, which proves SPI, the crystal and
 * the bit timing all work without needing another node to answer.
 */
class Mcp2515 {
public:
    static constexpr uint32_t kDefaultSpiHz = 10 * 1000 * 1000;  /* chip max */

    enum class Mode : uint8_t {
        Normal     = 0x00,
        Sleep      = 0x20,
        Loopback   = 0x40,
        ListenOnly = 0x60,
        Config     = 0x80,
    };

    struct Frame {
        uint32_t id       = 0;
        bool     extended = false;   /* 29-bit id rather than 11-bit */
        bool     rtr      = false;   /* remote transmission request */
        uint8_t  len      = 0;       /* 0..8 */
        uint8_t  data[8]  = {};
    };

    struct Config {
        spi_inst_t *spi = spi0;

        /* Defaults are the common MCP2515 wiring (SPI0 default pins). Each
         * board overrides these from its own board_config.hpp -- this header
         * is shared between boards, so it must not depend on any of them. */
        uint  sck   = 18;
        uint  tx    = 19;   /* -> MCP2515 SI */
        uint  rx    = 16;   /* <- MCP2515 SO */
        uint  cs    = 17;
        uint  reset = 20;
        uint  intr  = 21;

        uint32_t xtal_hz  = 8000000;    /* crystal fitted next to the MCP2515 */
        uint32_t bitrate  = 250000;     /* CAN bus bit rate */
        uint32_t spi_hz   = kDefaultSpiHz;
    };

    /* Result of check_wiring(): enough detail to name the faulty signal. */
    struct SpiCheck {
        uint8_t canstat        = 0xFF;   /* CANSTAT after reset, expect 0x80 */
        bool    in_config_mode = false;  /* chip reset into config mode */
        bool    readback_ok    = false;  /* every pattern survived the round trip */
        uint8_t sent[4]        = {0x00, 0xFF, 0xAA, 0x55};
        uint8_t got[4]         = {};
        bool    int_high       = false;  /* INT idle high (has a pull-up) */
        bool    cs_pin_high    = false;  /* CS pad really reaches a high level */
        bool    reset_pin_high = false;  /* RESET pad really reaches high */

        /* State of the SO/MISO line, measured by driving the RP2040's own
         * pulls and seeing whether the line follows. Distinguishes "nothing
         * is connected" from "something is holding it low". */
        bool    rx_follows_pulls = false;   /* floating: no driver present */
        bool    rx_driven_low    = false;   /* something holds it at 0 */
        bool    rx_driven_high   = false;   /* something holds it at 1 */
    };

    Mcp2515() = default;
    ~Mcp2515();

    Mcp2515(const Mcp2515 &) = delete;
    Mcp2515 &operator=(const Mcp2515 &) = delete;
    Mcp2515(Mcp2515 &&) = delete;
    Mcp2515 &operator=(Mcp2515 &&) = delete;

    /*
     * Reset the controller, program the bit timing and enter Normal mode.
     * Returns false if the xtal/bitrate pair has no valid segment split, or if
     * the chip does not answer over SPI -- the mode is read back and verified,
     * so a false here means the link is genuinely broken, not merely idle.
     */
    bool init(const Config &cfg);

    /*
     * Probe the SPI wiring without needing a successful init() -- this is the
     * tool for when init() returns false and you need to know which signal is
     * wrong. Sets up the pins itself, resets the chip, then writes four bit
     * patterns to a scratch register and reads them back, which exercises
     * MOSI, MISO, SCK and CS together. A read-only presence check cannot do
     * that: a MISO stuck high would still "read" a plausible value.
     *
     * Leaves the chip reset and in config mode, so call init() afterwards.
     */
    SpiCheck check_wiring(const Config &cfg);

    /* One-line plain-English reading of a SpiCheck, naming the likely signal
     * at fault. Returns a static string; never null. */
    static const char *diagnose(const SpiCheck &check);

    /*
     * Raw SPI transfer with the chip left DESELECTED, for the jumper test:
     * link the Pico's TX pin to its RX pin and whatever is sent comes back.
     * That isolates the RP2040 SPI block and the pin map from everything on
     * the far side of the connector -- if this echoes, the firmware is right
     * and the fault is the chip, its power, or its wiring.
     */
    void bus_echo(const Config &cfg, const uint8_t *tx, uint8_t *rx, size_t len);

    /* Loop a frame back internally with the bus disconnected. Restores the
     * previous mode. False means SPI or bit timing is wrong. */
    bool self_test();

    bool send(const Frame &frame);

    /* Non-blocking: false if no frame is waiting. */
    bool receive(Frame *frame);

    /* True when a received frame is waiting to be read. */
    bool available();

    /* INT is active low, so this reads the pin inverted. */
    bool interrupt_asserted() const;

    bool set_mode(Mode mode);
    Mode mode();

    uint8_t tx_error_count();
    uint8_t rx_error_count();
    uint8_t error_flags();      /* EFLG: bus-off, passive, overflow */

    bool is_initialised() const { return initialised_; }

    /* Exposed for tests: the CNF1..CNF3 triple for a xtal/bitrate pair.
     * Returns false if no valid split exists (e.g. 1 Mbit/s on an 8 MHz xtal). */
    static bool bit_timing(uint32_t xtal_hz, uint32_t bitrate,
                           uint8_t *cnf1, uint8_t *cnf2, uint8_t *cnf3);

private:
    void cs_select();
    void cs_deselect();
    uint8_t read_reg(uint8_t addr);
    void read_regs(uint8_t addr, uint8_t *buf, size_t len);
    void write_reg(uint8_t addr, uint8_t value);
    void write_regs(uint8_t addr, const uint8_t *buf, size_t len);
    void modify_reg(uint8_t addr, uint8_t mask, uint8_t value);
    void reset_chip();
    void setup_pins();

    Config cfg_{};
    bool   initialised_ = false;
};

#endif /* MCP2515_HPP */
