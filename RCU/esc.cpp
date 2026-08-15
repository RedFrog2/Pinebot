#include "esc.hpp"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

namespace {

/* The PWM counter is clocked at 1 MHz so a channel level is a pulse width in
 * microseconds and the wrap is the frame period. */
constexpr uint32_t kTicksPerUs = 1;
constexpr uint32_t kTickHz     = 1000000 * kTicksPerUs;

}  // namespace

Esc::~Esc()
{
    deinit();
}

void Esc::write_pulse(uint16_t us)
{
    pulse_us_ = us;
    pwm_set_chan_level(slice_, chan_, us * kTicksPerUs);
}

bool Esc::init(const Config &cfg)
{
    if (cfg.gpio >= NUM_BANK0_GPIOS) {
        return false;
    }
    if (cfg.freq_hz == 0) {
        return false;
    }

    /* Pulses must be ordered, and the deadband must leave room to travel on
     * both sides of neutral. */
    if (cfg.min_us + cfg.deadband_us >= cfg.neutral_us ||
        cfg.neutral_us + cfg.deadband_us >= cfg.max_us) {
        return false;
    }

    /* One frame has to hold the longest pulse and still fit the 16-bit
     * counter. Together these bound freq_hz to roughly 16..500 Hz. */
    uint32_t period_us = kTickHz / (kTicksPerUs * cfg.freq_hz);
    if (period_us <= cfg.max_us || period_us > 65536) {
        return false;
    }

    float div = static_cast<float>(clock_get_hz(clk_sys)) / static_cast<float>(kTickHz);
    if (div < 1.0f || div > 255.0f) {
        return false;   /* sys clock too far from the stock 125 MHz */
    }

    /* Validation passed, so release any pin we already hold before taking the
     * new one. */
    deinit();

    cfg_       = cfg;
    slice_     = pwm_gpio_to_slice_num(cfg.gpio);
    chan_      = pwm_gpio_to_channel(cfg.gpio);
    pulse_us_  = cfg.neutral_us;

    pwm_config pwm_cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&pwm_cfg, div);
    pwm_config_set_wrap(&pwm_cfg, static_cast<uint16_t>(period_us - 1));

    /* Load neutral before the slice runs so the very first edge the ESC sees
     * is a valid neutral pulse rather than a full-throttle one. */
    pwm_init(slice_, &pwm_cfg, false);
    write_pulse(cfg.neutral_us);
    gpio_set_function(cfg.gpio, GPIO_FUNC_PWM);
    pwm_set_enabled(slice_, true);

    initialised_ = true;
    return true;
}

void Esc::arm()
{
    if (!initialised_) {
        return;
    }
    write_pulse(cfg_.neutral_us);
    sleep_ms(cfg_.arm_ms);
    armed_ = true;
}

void Esc::set_throttle(float throttle)
{
    if (!initialised_) {
        return;
    }

    if (cfg_.invert) {
        throttle = -throttle;
    }

    uint16_t us;
    if (throttle > 0.0f) {
        if (throttle > 1.0f) {
            throttle = 1.0f;
        }
        uint16_t base = cfg_.neutral_us + cfg_.deadband_us;
        us = base + static_cast<uint16_t>(static_cast<float>(cfg_.max_us - base) * throttle + 0.5f);
    } else if (throttle < 0.0f) {
        if (throttle < -1.0f) {
            throttle = -1.0f;
        }
        uint16_t base = cfg_.neutral_us - cfg_.deadband_us;
        us = base - static_cast<uint16_t>(static_cast<float>(base - cfg_.min_us) * -throttle + 0.5f);
    } else {
        us = cfg_.neutral_us;   /* also catches NaN */
    }

    write_pulse(us);
}

void Esc::set_pulse_us(uint16_t us)
{
    if (!initialised_) {
        return;
    }
    if (us < cfg_.min_us) {
        us = cfg_.min_us;
    } else if (us > cfg_.max_us) {
        us = cfg_.max_us;
    }
    write_pulse(us);
}

void Esc::stop()
{
    if (!initialised_) {
        return;
    }
    write_pulse(cfg_.neutral_us);
}

void Esc::deinit()
{
    if (!initialised_) {
        return;
    }
    stop();
    pwm_set_enabled(slice_, false);
    gpio_set_function(cfg_.gpio, GPIO_FUNC_NULL);
    armed_       = false;
    initialised_ = false;
}
