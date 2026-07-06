#pragma once

#include "rd.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

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

// ── Master → slave ───────────────────────────────────────────────────────────

inline rd_Envelope make_multi_arm(uint32_t seq, uint32_t device_id, uint32_t action_id,
                                   const uint32_t *slots, pb_size_t slot_count,
                                   uint32_t fire_timeout_ms) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload = rd_Envelope_multi_arm_tag;
    rd_MultiArm &a = e.payload.multi_arm;
    a.device_id       = device_id;
    a.action_id       = action_id;
    a.fire_timeout_ms = fire_timeout_ms;
    a.slots_count     = slot_count < 4 ? slot_count : 4;
    for (pb_size_t i = 0; i < a.slots_count; i++) a.slots[i] = slots[i];
    return e;
}

inline rd_Envelope make_multi_fire(uint32_t seq, uint32_t device_id, uint32_t action_id,
                                    const uint32_t *slots, pb_size_t slot_count,
                                    const uint32_t *durations_ms) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload = rd_Envelope_multi_fire_tag;
    rd_MultiFire &f = e.payload.multi_fire;
    f.device_id        = device_id;
    f.action_id        = action_id;
    f.slots_count      = slot_count < 4 ? slot_count : 4;
    f.durations_ms_count = f.slots_count;
    for (pb_size_t i = 0; i < f.slots_count; i++) {
        f.slots[i]        = slots[i];
        f.durations_ms[i] = durations_ms[i];
    }
    return e;
}

// slots == nullptr / slot_count == 0  → reset all slots on the device.
inline rd_Envelope make_reset(uint32_t seq, uint32_t device_id, uint32_t action_id,
                               const uint32_t *slots = nullptr, pb_size_t slot_count = 0) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload = rd_Envelope_reset_tag;
    rd_Reset &r   = e.payload.reset;
    r.device_id   = device_id;
    r.action_id   = action_id;
    r.slots_count = slot_count < 4 ? slot_count : 4;
    for (pb_size_t i = 0; i < r.slots_count; i++) r.slots[i] = slots[i];
    return e;
}

// ── Slave → master ───────────────────────────────────────────────────────────
// Callers build the rd_ArmResponse / rd_FireResponse struct, then wrap it.

inline rd_Envelope make_arm_response(uint32_t seq, const rd_ArmResponse &r) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload      = rd_Envelope_arm_response_tag;
    e.payload.arm_response = r;
    return e;
}

inline rd_Envelope make_fire_response(uint32_t seq, const rd_FireResponse &r) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload       = rd_Envelope_fire_response_tag;
    e.payload.fire_response = r;
    return e;
}

inline rd_Envelope make_error(uint32_t seq, uint32_t device_id) {
    rd_Envelope e = rd_Envelope_init_zero;
    e.seq = seq;
    e.which_payload            = rd_Envelope_error_tag;
    e.payload.error.device_id  = device_id;
    return e;
}

} // namespace rd
