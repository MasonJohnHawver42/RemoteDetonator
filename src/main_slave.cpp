#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <esp_timer.h>
#include <LML.h>
#include "rd_codec.h"

#define VEXT_CTRL 36
#define OLED_SDA  17
#define OLED_SCL  18
#define OLED_RST  21
#define LORA_NSS  8
#define LORA_DIO1 14
#define LORA_RST  12
#define LORA_BUSY 13
#define LORA_SCK  9
#define LORA_MISO 11
#define LORA_MOSI 10

// Onboard LED on the Heltec WiFi LoRa 32 V3. Lit during any firing pulse.
#define LED_GPIO 35

// Slot firing lines. HIGH = MOSFET gate on → fire; LOW = safe/idle.
#define SLOT1_FIRE_GPIO 4
#define SLOT2_FIRE_GPIO 5
#define SLOT3_FIRE_GPIO 6
#define SLOT4_FIRE_GPIO 7

static constexpr uint32_t DEVICE_ID = 1;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);
SX1262        radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
LML::P2PAsync mac(radio, LORA_DIO1);

// ── Battery ──────────────────────────────────────────────────────────────────

#define VBAT_CTRL 37
#define VBAT_ADC   1
static constexpr float VBAT_SCALE = 238.7f;

static int readBatteryPercent() {
    pinMode(VBAT_CTRL, OUTPUT);
    digitalWrite(VBAT_CTRL, HIGH);
    delay(20);
    analogReadResolution(12);
    analogSetPinAttenuation(VBAT_ADC, ADC_11db);
    uint32_t raw = 0;
    for (int i = 0; i < 16; i++) raw += analogRead(VBAT_ADC);
    raw /= 16;
    digitalWrite(VBAT_CTRL, LOW);
    float vbat = raw / VBAT_SCALE;
    Serial.printf("[bat] raw=%lu vbat=%.2f\n", (unsigned long)raw, vbat);
    int pct = (int)((vbat - 3.3f) / (4.2f - 3.3f) * 100.0f + 0.5f);
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    return pct;
}

// ── Display ──────────────────────────────────────────────────────────────────

static SemaphoreHandle_t displayMutex;
static char dispLine1[32] = "waiting...";
static char dispLine2[32] = "";

static void setDisplay(const char *l1, const char *l2) {
    xSemaphoreTake(displayMutex, portMAX_DELAY);
    strncpy(dispLine1, l1, sizeof(dispLine1) - 1);
    strncpy(dispLine2, l2, sizeof(dispLine2) - 1);
    xSemaphoreGive(displayMutex);
}

void displayTask(void *) {
    display.begin();
    display.setFont(u8g2_font_ncenB08_tr);
    char l1[32], l2[32];
    int      batPct    = readBatteryPercent();
    uint32_t lastBatMs = millis();
    while (true) {
        if (millis() - lastBatMs >= 2000) {
            batPct    = readBatteryPercent();
            lastBatMs = millis();
        }
        xSemaphoreTake(displayMutex, portMAX_DELAY);
        strncpy(l1, dispLine1, sizeof(l1));
        strncpy(l2, dispLine2, sizeof(l2));
        xSemaphoreGive(displayMutex);
        display.clearBuffer();
        display.drawStr(0, 12, "[ SLAVE ]");
        char bat[8];
        snprintf(bat, sizeof(bat), "%d%%", batPct);
        display.drawStr(128 - display.getStrWidth(bat), 12, bat);
        display.drawStr(0, 32, l1);
        display.drawStr(0, 48, l2);
        display.sendBuffer();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void protocolTask(void *) {
    Serial.println("[slave] protocolTask start");
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    int state = radio.begin(915.0, 125.0, 9, 7,
                            RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8, 1.8);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[slave] radio FAILED state=%d\n", state);
        setDisplay("radio fail", "");
        vTaskDelete(NULL);
    }
    radio.setDio2AsRfSwitch(true);
    mac.init();
    Serial.println("[slave] mac init OK — polling");

    while (true) {
        mac.poll();
    }
}

// ── Slots ────────────────────────────────────────────────────────────────────
//
//   IDLE  — on MultiArm(this slot) → ARMED.
//   ARMED — arm window open (fire_timeout_ms from the Arm command);
//           on MultiFire → FIRED; on timeout → IDLE.
//   FIRED — slot has been pulsed; refuses re-arm until Reset.

enum class SlotState : uint8_t { Idle, Armed, Fired };

struct Slot {
    uint32_t      id;
    uint8_t       gpio;
    SlotState     state;
    uint32_t      action;
    TimerHandle_t armTimer;
};

static Slot s_slots[] = {
    { 1, SLOT1_FIRE_GPIO, SlotState::Idle, 0, nullptr },
    { 2, SLOT2_FIRE_GPIO, SlotState::Idle, 0, nullptr },
    { 3, SLOT3_FIRE_GPIO, SlotState::Idle, 0, nullptr },
    { 4, SLOT4_FIRE_GPIO, SlotState::Idle, 0, nullptr },
};
static constexpr size_t NUM_SLOTS = sizeof(s_slots) / sizeof(s_slots[0]);

static Slot *findSlot(uint32_t id) {
    for (size_t i = 0; i < NUM_SLOTS; i++)
        if (s_slots[i].id == id) return &s_slots[i];
    return nullptr;
}

// ── Internal events ──────────────────────────────────────────────────────────

enum class EvKind : uint8_t { RxEnvelope, LmlError, ArmTimeout };

struct AppEvent {
    EvKind kind;
    union {
        struct { rd_Envelope env; float rssi; } rx;
        LMLError err;
        size_t   slotIdx;
    };
};

// One fire job per MultiFire command. fireTask sequences each slot as fast as
// possible (pulse → immediately next slot), then sends a single FireResponse.
struct MultiFireJob {
    size_t    slot_indices[NUM_SLOTS];  // indices into s_slots[] to pulse
    uint32_t  durations_ms[NUM_SLOTS];  // pulse duration per slot from MultiFire
    pb_size_t slot_count;
    uint32_t  failed_slots[NUM_SLOTS];  // slot IDs that were not Armed
    pb_size_t failed_count;
    uint32_t  seq;
    uint32_t  action_id;
    uint32_t  device_id;
};

static QueueHandle_t s_evQueue;
static QueueHandle_t s_fireQueue;

static uint8_t s_txBuf[128];

static bool trySendOnce(const rd_Envelope &env) {
    size_t len = rd::encode(env, s_txBuf, sizeof(s_txBuf));
    if (len == 0) { Serial.println("[slave] encode failed"); return false; }
    return mac.queue_tx(s_txBuf, len) == LMLError::OK;
}

static void refreshStatus() {
    const char *nm[] = { "I", "A", "F" };
    char l2[32];
    int  n = 0;
    l2[0] = '\0';
    for (size_t i = 0; i < NUM_SLOTS && n < (int)sizeof(l2) - 6; i++) {
        n += snprintf(l2 + n, sizeof(l2) - n, "S%lu:%s ",
                      (unsigned long)s_slots[i].id,
                      nm[(uint8_t)s_slots[i].state]);
    }
    char l1[32];
    snprintf(l1, sizeof(l1), "dev %lu", (unsigned long)DEVICE_ID);
    setDisplay(l1, l2);
}

static void armTimerCb(TimerHandle_t t) {
    AppEvent e;
    e.kind    = EvKind::ArmTimeout;
    e.slotIdx = (size_t)(uintptr_t)pvTimerGetTimerID(t);
    xQueueSend(s_evQueue, &e, 0);
}

// ── Message handlers ─────────────────────────────────────────────────────────

static void onMultiArm(const rd_MultiArm &a, uint32_t seq) {
    if (a.device_id != DEVICE_ID) return;

    rd_ArmResponse resp = {};
    resp.device_id = DEVICE_ID;
    resp.action_id = a.action_id;

    for (pb_size_t i = 0; i < a.slots_count; i++) {
        uint32_t slot_id = a.slots[i];
        Slot *s = findSlot(slot_id);
        if (!s) {
            if (resp.slots_failed_count < 4)
                resp.slots_failed[resp.slots_failed_count++] = slot_id;
            continue;
        }
        if (s->state == SlotState::Fired) {
            if (resp.slots_already_fired_count < 4)
                resp.slots_already_fired[resp.slots_already_fired_count++] = slot_id;
            continue;
        }
        // IDLE or re-ARM: store action, open arm window with the master-supplied timeout.
        s->action = a.action_id;
        s->state  = SlotState::Armed;
        // xTimerChangePeriod restarts the timer (or starts it if stopped).
        xTimerChangePeriod(s->armTimer, pdMS_TO_TICKS(a.fire_timeout_ms), 0);
        if (resp.slots_armed_count < 4)
            resp.slots_armed[resp.slots_armed_count++] = slot_id;
        Serial.printf("[slave] slot %lu ARMED act %lu tmo=%lums\n",
                      (unsigned long)slot_id, (unsigned long)a.action_id,
                      (unsigned long)a.fire_timeout_ms);
    }

    resp.success = (resp.slots_armed_count > 0 &&
                    resp.slots_failed_count == 0 &&
                    resp.slots_already_fired_count == 0);

    if (!trySendOnce(rd::make_arm_response(seq, resp)))
        Serial.println("[slave] ArmResponse tx failed");

    refreshStatus();
}

static void onMultiFire(const rd_MultiFire &f, uint32_t seq) {
    if (f.device_id != DEVICE_ID) return;

    MultiFireJob job = {};
    job.seq       = seq;
    job.action_id = f.action_id;
    job.device_id = f.device_id;

    for (pb_size_t i = 0; i < f.slots_count; i++) {
        uint32_t slot_id = f.slots[i];
        Slot *s = findSlot(slot_id);
        if (!s || s->state != SlotState::Armed) {
            if (job.failed_count < NUM_SLOTS)
                job.failed_slots[job.failed_count++] = slot_id;
            continue;
        }
        // Mark fired now (owned by appTask) before handing to fireTask.
        s->state = SlotState::Fired;
        xTimerStop(s->armTimer, 0);
        if (job.slot_count < NUM_SLOTS) {
            job.slot_indices[job.slot_count]  = (size_t)(s - s_slots);
            job.durations_ms[job.slot_count]  = (i < f.durations_ms_count && f.durations_ms[i] > 0)
                                                  ? f.durations_ms[i] : 1200u;
            job.slot_count++;
        }
    }

    refreshStatus();

    if (job.slot_count == 0) {
        // Nothing to pulse; respond immediately without queuing.
        rd_FireResponse resp = {};
        resp.device_id = f.device_id;
        resp.action_id = f.action_id;
        resp.success   = false;
        for (pb_size_t i = 0; i < job.failed_count && resp.slots_failed_count < 4; i++)
            resp.slots_failed[resp.slots_failed_count++] = job.failed_slots[i];
        trySendOnce(rd::make_fire_response(seq, resp));
        return;
    }

    // Hand off to the high-priority fire task which pulses and sends FireResponse.
    if (xQueueSend(s_fireQueue, &job, 0) != pdTRUE)
        Serial.println("[slave] fire queue full — job dropped");
}

static void resetSlot(Slot &s) {
    xTimerStop(s.armTimer, 0);
    digitalWrite(s.gpio, LOW);
    s.state  = SlotState::Idle;
    s.action = 0;
}

static void onReset(const rd_Reset &r) {
    if (r.device_id != DEVICE_ID) return;
    if (r.slots_count == 0) {
        for (size_t i = 0; i < NUM_SLOTS; i++) resetSlot(s_slots[i]);
        Serial.println("[slave] RESET all slots → IDLE");
    } else {
        for (pb_size_t i = 0; i < r.slots_count; i++) {
            Slot *s = findSlot(r.slots[i]);
            if (!s) continue;
            resetSlot(*s);
            Serial.printf("[slave] RESET slot %lu → IDLE\n", (unsigned long)r.slots[i]);
        }
    }
    refreshStatus();
}

// ── Firing task ──────────────────────────────────────────────────────────────
// Pinned to core 1 at priority 4 (highest app priority) so the pulse timing
// is not preempted by the radio or app tasks.
//
// For each job: pulse each slot in order — HIGH for FIRE_PULSE_US, then
// immediately LOW and on to the next slot. No gap between slots. One
// FireResponse is sent after all slots are done.

static constexpr int64_t FIRE_DEBUG_GAP_US = 200000;  // 200 ms LOW between slots (visual break)

void fireTask(void *) {
    MultiFireJob job;
    while (xQueueReceive(s_fireQueue, &job, portMAX_DELAY) == pdTRUE) {

        rd_FireResponse resp = {};
        resp.device_id = job.device_id;
        resp.action_id = job.action_id;

        int64_t seq_start = esp_timer_get_time();
        for (pb_size_t i = 0; i < job.slot_count; i++) {
            Slot &s = s_slots[job.slot_indices[i]];

            int64_t slot_start = esp_timer_get_time();
            Serial.printf("[fire] slot %lu start  seq+%lldus\n",
                          (unsigned long)s.id, slot_start - seq_start);

            int64_t dur_us = (int64_t)job.durations_ms[i] * 1000;
            digitalWrite(LED_GPIO, HIGH);
            digitalWrite(s.gpio, HIGH);
            int64_t t0 = esp_timer_get_time();
            while (esp_timer_get_time() - t0 < dur_us) {}
            digitalWrite(s.gpio, LOW);
            digitalWrite(LED_GPIO, LOW);

            int64_t pulse_done = esp_timer_get_time();
            Serial.printf("[fire] slot %lu done   pulse=%lldus (req %lu ms)\n",
                          (unsigned long)s.id, pulse_done - t0, (unsigned long)job.durations_ms[i]);

            // Debug gap: hold LOW for FIRE_DEBUG_GAP_US so the boundary between
            // consecutive slot pulses is visually distinct on the GPIO lines.
            if (i + 1 < job.slot_count) {
                while (esp_timer_get_time() - pulse_done < FIRE_DEBUG_GAP_US) {}
                Serial.printf("[fire] gap done  next slot in %uus\n",
                              (unsigned)FIRE_DEBUG_GAP_US);
            }

            if (resp.slots_fired_count < 4)
                resp.slots_fired[resp.slots_fired_count++] = s.id;
        }
        Serial.printf("[fire] sequence complete total=%lldus slots=%u\n",
                      esp_timer_get_time() - seq_start, (unsigned)job.slot_count);

        // Include any pre-failed slots from onMultiFire.
        for (pb_size_t i = 0; i < job.failed_count && resp.slots_failed_count < 4; i++)
            resp.slots_failed[resp.slots_failed_count++] = job.failed_slots[i];

        resp.success = (resp.slots_fired_count > 0 && resp.slots_failed_count == 0);

        if (!trySendOnce(rd::make_fire_response(job.seq, resp)))
            Serial.println("[slave] FireResponse tx failed");
    }
}

// ── Pump: LML rx_queue → internal s_evQueue ─────────────────────────────────

void pumpTask(void *) {
    LML::Event lml;
    AppEvent   appEv;
    while (true) {
        xQueueReceive(mac.rx_queue, &lml, portMAX_DELAY);
        if (lml.tag == LML::Event::Tag::Error) {
            appEv.kind = EvKind::LmlError;
            appEv.err  = lml.err;
        } else {
            appEv.rx.rssi = lml.msg.rssi;
            if (rd::decode(lml.msg.data, lml.msg.len, appEv.rx.env)) {
                appEv.kind = EvKind::RxEnvelope;
            } else {
                Serial.printf("[slave] pump: decode failed len=%u\n", lml.msg.len);
                appEv.kind = EvKind::LmlError;
                appEv.err  = LMLError::BAD_LENGTH;
            }
        }
        xQueueSend(s_evQueue, &appEv, portMAX_DELAY);
    }
}

// ── App: drives the slot state machines ──────────────────────────────────────

void appTask(void *) {
    Serial.println("[slave] appTask start");
    refreshStatus();

    AppEvent ev;
    while (true) {
        if (xQueueReceive(s_evQueue, &ev, portMAX_DELAY) != pdTRUE) continue;

        switch (ev.kind) {
            case EvKind::ArmTimeout: {
                Slot &s = s_slots[ev.slotIdx];
                if (s.state == SlotState::Armed) {
                    Serial.printf("[slave] slot %lu arm window expired → IDLE\n",
                                  (unsigned long)s.id);
                    s.state = SlotState::Idle;
                    refreshStatus();
                }
                break;
            }
            case EvKind::LmlError:
                Serial.printf("[slave] link err=%d — ignoring\n", (int)ev.err);
                break;
            case EvKind::RxEnvelope: {
                const rd_Envelope &e = ev.rx.env;
                switch (e.which_payload) {
                    case rd_Envelope_ping_tag:
                        trySendOnce(rd::make_pong(e.seq));
                        break;
                    case rd_Envelope_multi_arm_tag:
                        onMultiArm(e.payload.multi_arm, e.seq);
                        break;
                    case rd_Envelope_multi_fire_tag:
                        onMultiFire(e.payload.multi_fire, e.seq);
                        break;
                    case rd_Envelope_reset_tag:
                        onReset(e.payload.reset);
                        break;
                    default:
                        Serial.printf("[slave] ignored variant=%d\n", e.which_payload);
                        break;
                }
                break;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("[slave] boot");

    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, LOW);
    delay(100);

    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, LOW);

    for (size_t i = 0; i < NUM_SLOTS; i++) {
        pinMode(s_slots[i].gpio, OUTPUT);
        digitalWrite(s_slots[i].gpio, LOW);
        s_slots[i].state    = SlotState::Idle;
        // Initial timer period of 10 s; overwritten by fire_timeout_ms at arm time.
        s_slots[i].armTimer = xTimerCreate("arm", pdMS_TO_TICKS(10000),
                                           pdFALSE, (void *)(uintptr_t)i,
                                           armTimerCb);
    }

    displayMutex = xSemaphoreCreateMutex();
    s_evQueue    = xQueueCreate(8, sizeof(AppEvent));
    s_fireQueue  = xQueueCreate(1, sizeof(MultiFireJob));

    xTaskCreatePinnedToCore(protocolTask, "lml_s", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(pumpTask,     "pump",  4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(appTask,      "app_s", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(fireTask,     "fire",  4096, NULL, 4, NULL, 1);
    xTaskCreate(displayTask, "display",   4096, NULL, 1, NULL);

    Serial.println("[slave] tasks created");
}

void loop() { vTaskDelay(portMAX_DELAY); }
