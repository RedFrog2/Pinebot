#ifndef ROV_CAN_PROTOCOL_HPP
#define ROV_CAN_PROTOCOL_HPP

#include <cstdint>

/*
 * CAN message contract between the ROV boards. Included by BOTH firmwares, so
 * they cannot drift apart -- change it here and both ends move together.
 *
 *      topside --LoRa--> [LCU] --CAN--> [RCU] --PWM--> ESC + servos
 *
 * Depends on nothing but <cstdint>: no Pico SDK, no board headers. That keeps
 * it host-compilable, so the encode/decode pair can be unit tested off-target.
 *
 * Wire format is little-endian fixed point rather than float: exact, half the
 * bytes, and no surprises if a node is ever a different architecture.
 *
 * FAIL-SILENT RULE: the RCU can only observe CAN silence -- it cannot tell
 * "commanded zero" from "the radio died and the LCU is repeating stale data".
 * So when the LoRa link goes quiet the LCU must STOP SENDING kIdCommand
 * entirely. Do not hold the last value and do not stream zeros; going silent
 * is what lets the RCU's timeout neutral the motor.
 */
namespace rov::can {

/* Lower IDs win arbitration, so commands outrank telemetry by construction. */
constexpr uint32_t kIdCommand = 0x100;   /* LCU -> RCU, 20 Hz */
constexpr uint32_t kIdStatus  = 0x200;   /* RCU -> LCU,  5 Hz */

constexpr uint8_t kDlc = 8;

constexpr uint32_t kCommandRateHz = 20;
constexpr uint32_t kStatusRateHz  = 5;

/* 20 missed frames of margin, so ordinary bus contention never trips it. */
constexpr uint32_t kCommandTimeoutMs = 1000;

constexpr int kServoCount = 4;

/* Command flags (byte 7). */
constexpr uint8_t kFlagEnable = 1u << 0;   /* clear = treat as stop */
constexpr uint8_t kFlagEstop  = 1u << 1;   /* set = stop now, ignore values */

/* Status flags (byte 4). */
constexpr uint8_t kStatusArmed    = 1u << 0;
constexpr uint8_t kStatusFailsafe = 1u << 1;   /* timeout tripped */
constexpr uint8_t kStatusEscOk    = 1u << 2;
constexpr uint8_t kStatusCanOk    = 1u << 3;

/* Fixed-point scales. The thruster gets 16 bits because throttle wants to be
 * smooth; the servos get 8, which is 254 steps over a 1000 us travel, i.e.
 * ~3.9 us per step -- at or below the deadband of a typical analog servo. */
constexpr int16_t kThrusterScale = 1000;
constexpr int8_t  kServoScale    = 127;

struct Command {
    float   thruster = 0.0f;              /* -1.0 .. +1.0 */
    float   servo[kServoCount] = {};      /* -1.0 .. +1.0, 0 = centre */
    uint8_t seq   = 0;                    /* increments per frame, wraps */
    uint8_t flags = 0;
};

struct Status {
    float    thruster = 0.0f;   /* actually applied */
    uint16_t pulse_us = 0;      /* ESC pulse width */
    uint8_t  flags    = 0;
    uint8_t  last_seq = 0;      /* last command sequence the RCU saw */
    uint8_t  can_eflg = 0;      /* MCP2515 EFLG */
    uint8_t  can_tec  = 0;      /* MCP2515 transmit error counter */
};

namespace detail {

inline float clamp_unit(float v)
{
    if (v > 1.0f) return 1.0f;
    if (v < -1.0f) return -1.0f;
    if (!(v == v)) return 0.0f;   /* NaN -> centre, never a stale value */
    return v;
}

inline int32_t round_to_int(float v)
{
    return static_cast<int32_t>(v < 0.0f ? v - 0.5f : v + 0.5f);
}

inline void put_i16(uint8_t *p, int16_t v)
{
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

inline int16_t get_i16(const uint8_t *p)
{
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                (static_cast<uint16_t>(p[1]) << 8));
}

}  // namespace detail

/*
 * Command frame layout (8 bytes)
 *
 *      0-1  int16  thruster, -1000 .. +1000
 *      2-5  int8   servo 0..3, -127 .. +127
 *      6    uint8  sequence counter
 *      7    uint8  flags
 */
inline void encode(const Command &cmd, uint8_t out[kDlc])
{
    detail::put_i16(out, static_cast<int16_t>(detail::round_to_int(
        detail::clamp_unit(cmd.thruster) * kThrusterScale)));
    for (int i = 0; i < kServoCount; i++) {
        out[2 + i] = static_cast<uint8_t>(static_cast<int8_t>(detail::round_to_int(
            detail::clamp_unit(cmd.servo[i]) * kServoScale)));
    }
    out[6] = cmd.seq;
    out[7] = cmd.flags;
}

inline void decode(const uint8_t in[kDlc], Command *cmd)
{
    cmd->thruster = detail::clamp_unit(
        static_cast<float>(detail::get_i16(in)) / kThrusterScale);
    for (int i = 0; i < kServoCount; i++) {
        cmd->servo[i] = detail::clamp_unit(
            static_cast<float>(static_cast<int8_t>(in[2 + i])) / kServoScale);
    }
    cmd->seq   = in[6];
    cmd->flags = in[7];
}

/*
 * Status frame layout (8 bytes)
 *
 *      0-1  int16   applied thruster, -1000 .. +1000
 *      2-3  uint16  ESC pulse width in microseconds
 *      4    uint8   status flags
 *      5    uint8   last command sequence seen
 *      6    uint8   MCP2515 EFLG
 *      7    uint8   MCP2515 TEC
 */
inline void encode(const Status &st, uint8_t out[kDlc])
{
    detail::put_i16(out, static_cast<int16_t>(detail::round_to_int(
        detail::clamp_unit(st.thruster) * kThrusterScale)));
    out[2] = static_cast<uint8_t>(st.pulse_us & 0xFF);
    out[3] = static_cast<uint8_t>((st.pulse_us >> 8) & 0xFF);
    out[4] = st.flags;
    out[5] = st.last_seq;
    out[6] = st.can_eflg;
    out[7] = st.can_tec;
}

inline void decode(const uint8_t in[kDlc], Status *st)
{
    st->thruster = detail::clamp_unit(
        static_cast<float>(detail::get_i16(in)) / kThrusterScale);
    st->pulse_us = static_cast<uint16_t>(static_cast<uint16_t>(in[2]) |
                                         (static_cast<uint16_t>(in[3]) << 8));
    st->flags    = in[4];
    st->last_seq = in[5];
    st->can_eflg = in[6];
    st->can_tec  = in[7];
}

}  // namespace rov::can

#endif /* ROV_CAN_PROTOCOL_HPP */
