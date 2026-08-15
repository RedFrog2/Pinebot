#include "mcp2515.hpp"

#include <cstring>

#include "hardware/gpio.h"

namespace {

/* SPI instructions (datasheet section 12). */
constexpr uint8_t kCmdWrite     = 0x02;
constexpr uint8_t kCmdRead      = 0x03;
constexpr uint8_t kCmdBitModify = 0x05;
constexpr uint8_t kCmdRtsTx0    = 0x81;
constexpr uint8_t kCmdReset     = 0xC0;

/* Registers. */
constexpr uint8_t kRegCanstat  = 0x0E;
constexpr uint8_t kRegCanctrl  = 0x0F;
constexpr uint8_t kRegTec      = 0x1C;
constexpr uint8_t kRegRec      = 0x1D;
constexpr uint8_t kRegCnf3     = 0x28;
constexpr uint8_t kRegCnf2     = 0x29;
constexpr uint8_t kRegCnf1     = 0x2A;
constexpr uint8_t kRegCaninte  = 0x2B;
constexpr uint8_t kRegCanintf  = 0x2C;
constexpr uint8_t kRegEflg     = 0x2D;
constexpr uint8_t kRegTxb0ctrl = 0x30;
constexpr uint8_t kRegTxb0sidh = 0x31;
constexpr uint8_t kRegRxb0ctrl = 0x60;
constexpr uint8_t kRegRxb0sidh = 0x61;
constexpr uint8_t kRegRxb1ctrl = 0x70;

constexpr uint8_t kModeMask  = 0xE0;   /* REQOP / OPMOD live in bits 7:5 */
constexpr uint8_t kIntfRx0if = 0x01;
constexpr uint8_t kIntfRx1if = 0x02;
constexpr uint8_t kTxreq     = 0x08;

/* Segment limits from the datasheet, in time quanta. */
constexpr uint32_t kSyncSeg    = 1;
constexpr uint32_t kMinPropSeg = 1, kMaxPropSeg = 8;
constexpr uint32_t kMinPhSeg1  = 1, kMaxPhSeg1  = 8;
constexpr uint32_t kMinPhSeg2  = 2, kMaxPhSeg2  = 8;  /* PS2 >= 2 */
constexpr uint32_t kMinTotalTq = 8, kMaxTotalTq = 25;

}  // namespace

/*
 * Solve for the bit timing rather than carrying a lookup table, so an unusual
 * crystal still works and every combination gets the same sample point.
 *
 *      TQ       = 2 * (BRP + 1) / F_osc
 *      bit time = (SyncSeg + PropSeg + PhSeg1 + PhSeg2) * TQ
 *
 * We aim the sample point at 87.5% (the CiA recommendation for rates up to
 * 500 kbit/s) and take the closest achievable split.
 */
bool Mcp2515::bit_timing(uint32_t xtal_hz, uint32_t bitrate,
                         uint8_t *cnf1, uint8_t *cnf2, uint8_t *cnf3)
{
    if (xtal_hz == 0 || bitrate == 0) {
        return false;
    }

    for (uint32_t brp = 0; brp < 64; brp++) {
        /* Total quanta per bit for this prescaler, rejecting anything that
         * does not divide exactly -- an inexact bit rate is not usable. */
        uint32_t tq_hz = xtal_hz / (2 * (brp + 1));
        if (tq_hz % bitrate != 0) {
            continue;
        }
        uint32_t total = tq_hz / bitrate;
        if (total < kMinTotalTq || total > kMaxTotalTq) {
            continue;
        }

        /* Sample point 87.5% => PS2 is the last 12.5% of the bit, at least
         * kMinPhSeg2 quanta. */
        uint32_t ps2 = (total * 125 + 999) / 1000;
        if (ps2 < kMinPhSeg2) {
            ps2 = kMinPhSeg2;
        }
        if (ps2 > kMaxPhSeg2) {
            ps2 = kMaxPhSeg2;
        }

        uint32_t remaining = total - kSyncSeg - ps2;   /* PropSeg + PhSeg1 */
        if (remaining < kMinPropSeg + kMinPhSeg1) {
            continue;
        }

        /* Split the remainder, keeping PhSeg1 >= PropSeg and both in range. */
        uint32_t ph1  = (remaining + 1) / 2;
        uint32_t prop = remaining - ph1;
        if (ph1 > kMaxPhSeg1) {
            ph1  = kMaxPhSeg1;
            prop = remaining - ph1;
        }
        if (prop > kMaxPropSeg || prop < kMinPropSeg || ph1 < kMinPhSeg1) {
            continue;
        }

        /* SJW of 1 TQ is always legal and is never larger than PS2. */
        *cnf1 = static_cast<uint8_t>(brp & 0x3F);
        /* BTLMODE=1 so PS2 comes from CNF3; SAM=0 (sample once). */
        *cnf2 = static_cast<uint8_t>(0x80 | ((ph1 - 1) << 3) | (prop - 1));
        *cnf3 = static_cast<uint8_t>(ps2 - 1);
        return true;
    }

    return false;
}

Mcp2515::~Mcp2515()
{
    if (initialised_) {
        set_mode(Mode::Sleep);
    }
}

void Mcp2515::cs_select()
{
    gpio_put(cfg_.cs, 0);
    asm volatile("nop \n nop \n nop");
}

void Mcp2515::cs_deselect()
{
    asm volatile("nop \n nop \n nop");
    gpio_put(cfg_.cs, 1);
}

uint8_t Mcp2515::read_reg(uint8_t addr)
{
    uint8_t value = 0;
    read_regs(addr, &value, 1);
    return value;
}

void Mcp2515::read_regs(uint8_t addr, uint8_t *buf, size_t len)
{
    const uint8_t header[2] = {kCmdRead, addr};
    cs_select();
    spi_write_blocking(cfg_.spi, header, 2);
    spi_read_blocking(cfg_.spi, 0x00, buf, len);
    cs_deselect();
}

void Mcp2515::write_reg(uint8_t addr, uint8_t value)
{
    write_regs(addr, &value, 1);
}

void Mcp2515::write_regs(uint8_t addr, const uint8_t *buf, size_t len)
{
    const uint8_t header[2] = {kCmdWrite, addr};
    cs_select();
    spi_write_blocking(cfg_.spi, header, 2);
    spi_write_blocking(cfg_.spi, buf, len);
    cs_deselect();
}

void Mcp2515::modify_reg(uint8_t addr, uint8_t mask, uint8_t value)
{
    const uint8_t buf[4] = {kCmdBitModify, addr, mask, value};
    cs_select();
    spi_write_blocking(cfg_.spi, buf, 4);
    cs_deselect();
}

void Mcp2515::reset_chip()
{
    /* Hardware reset first, then the SPI reset command, so the chip is in a
     * known state even if it was mid-transfer when we rebooted. */
    gpio_put(cfg_.reset, 0);
    sleep_us(10);
    gpio_put(cfg_.reset, 1);
    sleep_ms(5);

    const uint8_t cmd = kCmdReset;
    cs_select();
    spi_write_blocking(cfg_.spi, &cmd, 1);
    cs_deselect();
    sleep_ms(10);   /* datasheet: chip needs time to re-enter config mode */
}

void Mcp2515::setup_pins()
{
    spi_init(cfg_.spi, cfg_.spi_hz);
    gpio_set_function(cfg_.sck, GPIO_FUNC_SPI);
    gpio_set_function(cfg_.tx, GPIO_FUNC_SPI);
    gpio_set_function(cfg_.rx, GPIO_FUNC_SPI);

    /* CS and RESET are plain outputs, both idle high (active low).
     * Set the output level BEFORE switching to output: gpio_init() clears the
     * output register, so driving first would glitch both lines low -- a
     * spurious chip select and an unasked-for reset pulse on every setup. */
    gpio_init(cfg_.cs);
    gpio_put(cfg_.cs, 1);
    gpio_set_dir(cfg_.cs, GPIO_OUT);
    gpio_init(cfg_.reset);
    gpio_put(cfg_.reset, 1);
    gpio_set_dir(cfg_.reset, GPIO_OUT);

    /* INT is open drain on the MCP2515, so it needs a pull-up. */
    gpio_init(cfg_.intr);
    gpio_set_dir(cfg_.intr, GPIO_IN);
    gpio_pull_up(cfg_.intr);
}

Mcp2515::SpiCheck Mcp2515::check_wiring(const Config &cfg)
{
    SpiCheck result;

    cfg_ = cfg;
    setup_pins();
    reset_chip();

    /* Reading a pin configured as an output returns the real pad level, so
     * this catches a control line shorted low or loaded down externally --
     * we are driving both high at this point. */
    gpio_put(cfg_.cs, 1);
    gpio_put(cfg_.reset, 1);
    sleep_us(100);
    result.cs_pin_high    = gpio_get(cfg_.cs) != 0;
    result.reset_pin_high = gpio_get(cfg_.reset) != 0;

    result.canstat        = read_reg(kRegCanstat);
    result.in_config_mode = (result.canstat & kModeMask) ==
                            static_cast<uint8_t>(Mode::Config);
    result.int_high       = gpio_get(cfg_.intr) != 0;

    /*
     * TXB0SIDH is writable in any mode and is only latched when a send is
     * requested, so scribbling patterns on it is harmless. 0x00 and 0xFF catch
     * a line stuck low or high; 0xAA and 0x55 catch a shorted or swapped pair,
     * which a single pattern would miss.
     */
    result.readback_ok = true;
    for (int i = 0; i < 4; i++) {
        write_reg(kRegTxb0sidh, result.sent[i]);
        result.got[i] = read_reg(kRegTxb0sidh);
        if (result.got[i] != result.sent[i]) {
            result.readback_ok = false;
        }
    }
    write_reg(kRegTxb0sidh, 0x00);

    /*
     * If the readback failed, work out whether SO is floating or actively
     * held. Briefly take the pin away from the SPI block and let the RP2040's
     * internal pulls fight it: a floating line follows the pull, a driven line
     * does not. This separates "no chip / not wired" from "chip held in reset
     * or shorted", which look identical from the register reads alone.
     */
    if (!result.readback_ok) {
        /* CS must be ASSERTED for this to mean anything: the MCP2515
         * tri-states SO whenever it is deselected, so an unselected healthy
         * chip floats exactly like an absent one. Held low, the chip keeps
         * driving SO between transfers. */
        cs_select();
        gpio_set_function(cfg_.rx, GPIO_FUNC_SIO);
        gpio_set_dir(cfg_.rx, GPIO_IN);

        gpio_pull_up(cfg_.rx);
        sleep_us(100);
        const bool with_pullup = gpio_get(cfg_.rx);

        gpio_pull_down(cfg_.rx);
        sleep_us(100);
        const bool with_pulldown = gpio_get(cfg_.rx);

        gpio_disable_pulls(cfg_.rx);
        gpio_set_function(cfg_.rx, GPIO_FUNC_SPI);
        cs_deselect();

        result.rx_follows_pulls = with_pullup && !with_pulldown;
        result.rx_driven_low    = !with_pullup && !with_pulldown;
        result.rx_driven_high   = with_pullup && with_pulldown;
    }

    /* The probe left the chip reset, so any previous init is void. */
    initialised_ = false;
    return result;
}

void Mcp2515::bus_echo(const Config &cfg, const uint8_t *tx, uint8_t *rx, size_t len)
{
    cfg_ = cfg;
    setup_pins();

    /* CS stays high throughout: a fitted MCP2515 must not answer, so anything
     * that comes back arrived over the jumper and nowhere else. */
    gpio_put(cfg_.cs, 1);
    spi_write_read_blocking(cfg_.spi, tx, rx, len);

    initialised_ = false;
}

const char *Mcp2515::diagnose(const SpiCheck &check)
{
    bool all_zero = true, all_ones = true;
    for (int i = 0; i < 4; i++) {
        if (check.got[i] != 0x00) all_zero = false;
        if (check.got[i] != 0xFF) all_ones = false;
    }

    if (!check.reset_pin_high) {
        return "RESET will not go high: the line is shorted or loaded down, so "
               "the chip is held in reset with its outputs high-Z";
    }
    if (!check.cs_pin_high) {
        return "CS will not go high: the line is shorted low, so the chip is "
               "permanently selected";
    }
    if (all_zero && check.rx_follows_pulls) {
        return "SO floats even with CS asserted: no chip driving it -- "
               "unpowered, not fitted, or SO not landing on the Pico RX pin";
    }
    if (all_zero && check.rx_driven_low) {
        return "SO is actively held low with CS asserted: shorted to GND, or "
               "the chip is driving 0 (check RESET is high and OSC1 running)";
    }
    if (all_zero) {
        return "every read came back 0x00: chip unpowered, GND not shared, or "
               "SO not reaching the Pico RX pin";
    }
    if (all_ones) {
        return "every read came back 0xFF: MISO floating -- SO not connected, "
               "or CS never asserted (chip not selected)";
    }
    if (!check.in_config_mode) {
        return "chip responds but did not reset into config mode: check the "
               "RESET line and that the crystal is oscillating";
    }
    if (!check.readback_ok) {
        return "patterns came back corrupted: check SCK and SI (MOSI), look "
               "for a swapped SI/SO pair, or try a lower spi_hz";
    }
    return "SPI wiring good: CS, SCK, SI and SO all verified";
}

bool Mcp2515::init(const Config &cfg)
{
    uint8_t cnf1, cnf2, cnf3;
    if (!bit_timing(cfg.xtal_hz, cfg.bitrate, &cnf1, &cnf2, &cnf3)) {
        return false;   /* no valid segment split for this xtal/bitrate */
    }

    cfg_ = cfg;
    setup_pins();
    reset_chip();

    /* After reset the chip must be in config mode. If it is not, nothing is
     * answering on SPI and there is no point continuing. */
    if ((read_reg(kRegCanstat) & kModeMask) != static_cast<uint8_t>(Mode::Config)) {
        return false;
    }

    write_reg(kRegCnf1, cnf1);
    write_reg(kRegCnf2, cnf2);
    write_reg(kRegCnf3, cnf3);

    /* Accept every frame: masks off on both receive buffers, with rollover
     * from RXB0 into RXB1 so a burst does not drop frames. */
    write_reg(kRegRxb0ctrl, 0x60 | 0x04);
    write_reg(kRegRxb1ctrl, 0x60);

    write_reg(kRegCaninte, kIntfRx0if | kIntfRx1if);
    write_reg(kRegCanintf, 0x00);

    initialised_ = true;

    if (!set_mode(Mode::Normal)) {
        initialised_ = false;
        return false;
    }
    return true;
}

bool Mcp2515::set_mode(Mode mode)
{
    modify_reg(kRegCanctrl, kModeMask, static_cast<uint8_t>(mode));

    /* Verify: a mode change can be refused, e.g. Normal is not entered until
     * the chip sees an idle bus. */
    for (int i = 0; i < 20; i++) {
        if ((read_reg(kRegCanstat) & kModeMask) == static_cast<uint8_t>(mode)) {
            return true;
        }
        sleep_ms(1);
    }
    return false;
}

Mcp2515::Mode Mcp2515::mode()
{
    return static_cast<Mode>(read_reg(kRegCanstat) & kModeMask);
}

bool Mcp2515::send(const Frame &frame)
{
    if (!initialised_ || frame.len > 8) {
        return false;
    }

    /* TXB0SIDH..TXB0D7 written in one burst: id, dlc, then payload. */
    uint8_t buf[13] = {};
    if (frame.extended) {
        uint32_t id = frame.id & 0x1FFFFFFF;
        buf[0] = static_cast<uint8_t>(id >> 21);
        buf[1] = static_cast<uint8_t>(((id >> 13) & 0xE0) | 0x08 | ((id >> 16) & 0x03));
        buf[2] = static_cast<uint8_t>(id >> 8);
        buf[3] = static_cast<uint8_t>(id);
    } else {
        uint32_t id = frame.id & 0x7FF;
        buf[0] = static_cast<uint8_t>(id >> 3);
        buf[1] = static_cast<uint8_t>((id & 0x07) << 5);
    }
    buf[4] = static_cast<uint8_t>((frame.rtr ? 0x40 : 0x00) | frame.len);
    memcpy(&buf[5], frame.data, frame.len);

    write_regs(kRegTxb0sidh, buf, 5u + frame.len);

    /* Request to send, then wait for the buffer to drain. TXREQ clears when
     * the frame wins arbitration and is sent, or when it is aborted. */
    const uint8_t rts = kCmdRtsTx0;
    cs_select();
    spi_write_blocking(cfg_.spi, &rts, 1);
    cs_deselect();

    for (int i = 0; i < 100; i++) {
        if ((read_reg(kRegTxb0ctrl) & kTxreq) == 0) {
            return true;
        }
        sleep_ms(1);
    }
    return false;   /* no ack from any other node, or bus fault */
}

bool Mcp2515::available()
{
    if (!initialised_) {
        return false;
    }
    return (read_reg(kRegCanintf) & (kIntfRx0if | kIntfRx1if)) != 0;
}

bool Mcp2515::interrupt_asserted() const
{
    return initialised_ && !gpio_get(cfg_.intr);
}

bool Mcp2515::receive(Frame *frame)
{
    if (!initialised_ || frame == nullptr) {
        return false;
    }

    uint8_t flags = read_reg(kRegCanintf);
    uint8_t base;
    uint8_t flag;
    if (flags & kIntfRx0if) {
        base = kRegRxb0sidh;
        flag = kIntfRx0if;
    } else if (flags & kIntfRx1if) {
        base = kRegRxb1ctrl + 1;   /* RXB1SIDH */
        flag = kIntfRx1if;
    } else {
        return false;
    }

    uint8_t buf[13];
    read_regs(base, buf, 13);

    const bool extended = (buf[1] & 0x08) != 0;
    frame->extended = extended;
    if (extended) {
        frame->id = (static_cast<uint32_t>(buf[0]) << 21) |
                    (static_cast<uint32_t>(buf[1] & 0xE0) << 13) |
                    (static_cast<uint32_t>(buf[1] & 0x03) << 16) |
                    (static_cast<uint32_t>(buf[2]) << 8) |
                    static_cast<uint32_t>(buf[3]);
        frame->rtr = (buf[4] & 0x40) != 0;
    } else {
        frame->id = (static_cast<uint32_t>(buf[0]) << 3) |
                    (static_cast<uint32_t>(buf[1]) >> 5);
        frame->rtr = (buf[1] & 0x10) != 0;
    }

    frame->len = buf[4] & 0x0F;
    if (frame->len > 8) {
        frame->len = 8;
    }
    memcpy(frame->data, &buf[5], frame->len);

    /* Clear the flag so the buffer can take another frame. */
    modify_reg(kRegCanintf, flag, 0x00);
    return true;
}

bool Mcp2515::self_test()
{
    if (!initialised_) {
        return false;
    }

    const Mode previous = mode();
    if (!set_mode(Mode::Loopback)) {
        return false;
    }

    Frame tx{};
    tx.id  = 0x7FF;
    tx.len = 8;
    for (uint8_t i = 0; i < 8; i++) {
        tx.data[i] = static_cast<uint8_t>(0xA5 ^ i);
    }

    bool ok = send(tx);
    if (ok) {
        ok = false;
        for (int i = 0; i < 50 && !ok; i++) {
            Frame rx;
            if (receive(&rx)) {
                ok = rx.id == tx.id && rx.len == tx.len &&
                     memcmp(rx.data, tx.data, tx.len) == 0;
            } else {
                sleep_ms(1);
            }
        }
    }

    set_mode(previous);
    return ok;
}

uint8_t Mcp2515::tx_error_count()
{
    return initialised_ ? read_reg(kRegTec) : 0;
}

uint8_t Mcp2515::rx_error_count()
{
    return initialised_ ? read_reg(kRegRec) : 0;
}

uint8_t Mcp2515::error_flags()
{
    return initialised_ ? read_reg(kRegEflg) : 0;
}
