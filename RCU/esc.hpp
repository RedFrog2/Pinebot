#ifndef ESC_HPP
#define ESC_HPP

#include <cstdint>

#include "pico/stdlib.h"

/*
 * Driver for hobby RC brushed ESCs with a BEC, e.g. the Hobbywing QuicRun
 * WP-1080 / WP-860 used on the thrusters.
 *
 * The ESC is commanded with the usual RC servo pulse: a ~50 Hz pulse train
 * where the high time selects the output. 1500 us is neutral (motor stopped),
 * 1000 us is full reverse and 2000 us is full forward. The ESC ignores a small
 * deadband either side of neutral so a sloppy radio does not creep the motor.
 *
 * The BEC output of the ESC is a 5-6 V supply for the receiver side. Do NOT
 * back-feed it into VSYS if the Pico is already powered over USB, and tie the
 * ESC signal ground to the Pico ground.
 *
 * Usage:
 *
 *      Esc port;                       // may live at static scope
 *
 *      port.init(Esc::Config(14));     // claims the pin, emits neutral
 *      port.arm();                     // blocks while the ESC boots
 *      port.set_throttle(0.5f);        // half forward
 *
 * Construction is separate from init() so an Esc can be a global: the
 * constructor touches no hardware, init() does all of it and must run from
 * main() once the clocks are up.
 *
 * The ESC only arms if it sees neutral at power-up, so arm() must run before
 * any throttle command (see arm() for details).
 */
class Esc {
public:
    /* Pulse widths in microseconds. */
    static constexpr uint16_t kMinPulseUs     = 1000;   /* full reverse */
    static constexpr uint16_t kNeutralPulseUs = 1500;   /* stopped */
    static constexpr uint16_t kMaxPulseUs     = 2000;   /* full forward */

    /* The QuicRun ships with roughly this much deadband around neutral;
     * commands are mapped past it so a small throttle still turns the motor. */
    static constexpr uint16_t kDeadbandUs = 50;

    static constexpr uint16_t kFreqHz = 50;
    static constexpr uint16_t kArmMs  = 2000;

    struct Config {
        explicit Config(uint signal_gpio) : gpio(signal_gpio) {}

        uint     gpio;                          /* signal pin, driven as PWM */
        uint16_t min_us      = kMinPulseUs;     /* pulse at throttle -1.0 */
        uint16_t neutral_us  = kNeutralPulseUs; /* pulse at throttle  0.0 */
        uint16_t max_us      = kMaxPulseUs;     /* pulse at throttle +1.0 */
        uint16_t deadband_us = kDeadbandUs;     /* skipped either side of neutral */
        uint16_t freq_hz     = kFreqHz;         /* pulse rate, 16..500 Hz */
        uint16_t arm_ms      = kArmMs;          /* neutral hold time in arm() */
        bool     invert      = false;           /* swap forward/reverse for a
                                                   flipped thruster */
    };

    Esc() = default;
    ~Esc();

    /* An Esc owns its pin and PWM slice, so it cannot be copied or moved: a
     * second object releasing the same hardware would cut the motor. */
    Esc(const Esc &) = delete;
    Esc &operator=(const Esc &) = delete;
    Esc(Esc &&) = delete;
    Esc &operator=(Esc &&) = delete;

    /*
     * Claim the PWM hardware for cfg.gpio and start emitting a neutral pulse.
     * Returns false and changes nothing if the config is out of range. Calling
     * it on an already initialised Esc reconfigures it.
     *
     * Note that RP2040 PWM slices are shared between GPIO pairs (0/1, 2/3,
     * ...): two ESCs on a shared slice must use the same freq_hz, and
     * initialising the second one restarts the counter of the first.
     */
    bool init(const Config &cfg);

    /*
     * Hold neutral for cfg.arm_ms so the ESC completes its power-up self test.
     * Blocking. Throttle commands before this are ignored by the ESC anyway,
     * and a non-neutral signal at power-up puts it into its programming mode.
     */
    void arm();

    /* Throttle in [-1.0, +1.0]; out of range values are clamped. */
    void set_throttle(float throttle);

    /* Raw pulse width, clamped to [min_us, max_us]. Bypasses the deadband map. */
    void set_pulse_us(uint16_t us);

    /* Stop the motor. The QuicRun coasts to a stop rather than braking. */
    void stop();

    /* Stop the motor and release the pin. */
    void deinit();

    uint16_t pulse_us() const { return pulse_us_; }
    bool is_armed() const { return armed_; }
    bool is_initialised() const { return initialised_; }

private:
    void write_pulse(uint16_t us);

    Config   cfg_{0};
    uint     slice_       = 0;      /* PWM slice driving cfg_.gpio */
    uint     chan_        = 0;      /* PWM channel within the slice */
    uint16_t pulse_us_    = 0;      /* pulse currently on the pin */
    bool     armed_       = false;
    bool     initialised_ = false;
};

#endif /* ESC_HPP */
