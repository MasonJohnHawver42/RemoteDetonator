#pragma once

#include "rd.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include <stddef.h>
#include <stdint.h>

namespace rd {

// Encodes env into buf. Returns bytes written, or 0 on failure.
inline size_t encode(const rd_Envelope &env, uint8_t *buf, size_t cap) {
    pb_ostream_t s = pb_ostream_from_buffer(buf, cap);
    if (!pb_encode(&s, rd_Envelope_fields, &env)) return 0;
    return s.bytes_written;
}

// Decodes buf into env. Returns true on success.
inline bool decode(const uint8_t *buf, size_t len, rd_Envelope &env) {
    env = rd_Envelope_init_zero;
    pb_istream_t s = pb_istream_from_buffer(buf, len);
    return pb_decode(&s, rd_Envelope_fields, &env);
}

inline rd_Envelope make_ping(uint32_t seq) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload = rd_Envelope_ping_tag;
    return e;
}

inline rd_Envelope make_pong(uint32_t seq) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload = rd_Envelope_pong_tag;
    return e;
}

// ── Firing messages ──────────────────────────────────────────────────────────
// Arm/Armed/Disarmed/Fire/Fired/Misfired all share the (slot, device_id,
// action_id) shape, so a single private helper fills them in.

inline rd_Envelope make_arm(uint32_t seq, uint32_t slot, uint32_t device_id,
                            uint32_t action_id) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload = rd_Envelope_arm_tag;
    e.payload.arm.slot      = slot;
    e.payload.arm.device_id = device_id;
    e.payload.arm.action_id = action_id;
    return e;
}

inline rd_Envelope make_armed(uint32_t seq, uint32_t slot, uint32_t device_id,
                              uint32_t action_id) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload = rd_Envelope_armed_tag;
    e.payload.armed.slot      = slot;
    e.payload.armed.device_id = device_id;
    e.payload.armed.action_id = action_id;
    return e;
}

inline rd_Envelope make_disarmed(uint32_t seq, uint32_t slot, uint32_t device_id,
                                 uint32_t action_id) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload = rd_Envelope_disarmed_tag;
    e.payload.disarmed.slot      = slot;
    e.payload.disarmed.device_id = device_id;
    e.payload.disarmed.action_id = action_id;
    return e;
}

inline rd_Envelope make_fire(uint32_t seq, uint32_t slot, uint32_t device_id,
                             uint32_t action_id) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload = rd_Envelope_fire_tag;
    e.payload.fire.slot      = slot;
    e.payload.fire.device_id = device_id;
    e.payload.fire.action_id = action_id;
    return e;
}

inline rd_Envelope make_fired(uint32_t seq, uint32_t slot, uint32_t device_id,
                              uint32_t action_id) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload = rd_Envelope_fired_tag;
    e.payload.fired.slot      = slot;
    e.payload.fired.device_id = device_id;
    e.payload.fired.action_id = action_id;
    return e;
}

inline rd_Envelope make_misfired(uint32_t seq, uint32_t slot, uint32_t device_id,
                                 uint32_t action_id) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload = rd_Envelope_misfired_tag;
    e.payload.misfired.slot      = slot;
    e.payload.misfired.device_id = device_id;
    e.payload.misfired.action_id = action_id;
    return e;
}

// Master → slave slot reset. slot == 0 resets every slot on the device.
inline rd_Envelope make_reset(uint32_t seq, uint32_t slot, uint32_t device_id,
                              uint32_t action_id) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload = rd_Envelope_reset_tag;
    e.payload.reset.slot      = slot;
    e.payload.reset.device_id = device_id;
    e.payload.reset.action_id = action_id;
    return e;
}

inline rd_Envelope make_nack(uint32_t seq, uint32_t device_id) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload = rd_Envelope_nack_tag;
    e.payload.nack.device_id = device_id;
    return e;
}

inline rd_Envelope make_error(uint32_t seq, uint32_t device_id) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload = rd_Envelope_error_tag;
    e.payload.error.device_id = device_id;
    return e;
}

} // namespace rd
