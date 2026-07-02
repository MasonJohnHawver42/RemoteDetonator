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

// Onboard LED on the Heltec WiFi LoRa 32 V3. Lit alongside the firing line
// during a pulse as a visual "firing now" indicator (idles LOW).
#define LED_GPIO 35

// Slot 1's real firing line. GPIO 7 (3.3V logic) → 220R → gate of an
// IRLZ44N logic-level N-MOSFET (10k gate pulldown holds it off when LOW).
// HIGH = MOSFET on → ~4–5A through the nichrome → fire. LOW = safe/resting,
// matching the hardware pulldown. No inversion: HIGH = on, LOW = off.
#define SLOT1_FIRE_GPIO 7

// Compiled into the binary: this node's device id. Each slot drives a GPIO
// firing line that must idle LOW.
static constexpr uint32_t DEVICE_ID = 1;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);
SX1262        radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
LML::P2PAsync mac(radio, LORA_DIO1);

// ── Battery ──────────────────────────────────────────────────────────────────
// Heltec WiFi LoRa 32 V3: VBAT is sensed on GPIO1 through a resistor divider
// that is only powered while VBAT_CTRL (GPIO37) is driven LOW. Reads are
// throttled and the percentage cached so the divider isn't toggled every frame.

#define VBAT_CTRL 37
#define VBAT_ADC   1
// Empirical count→volts factor for the V3 divider: vbat[V] = raw / VBAT_SCALE.
// If the % is off, correct this against a multimeter using the [bat] raw= line.
static constexpr float VBAT_SCALE = 238.7f;

static int readBatteryPercent() {
    pinMode(VBAT_CTRL, OUTPUT);
    digitalWrite(VBAT_CTRL, HIGH);           // enable the divider (gates VBAT)
    delay(20);                               // let the tap settle
    analogReadResolution(12);
    analogSetPinAttenuation(VBAT_ADC, ADC_11db);
    uint32_t raw = 0;
    for (int i = 0; i < 16; i++) raw += analogRead(VBAT_ADC);
    raw /= 16;
    digitalWrite(VBAT_CTRL, LOW);            // disconnect to save current
    float vbat = raw / VBAT_SCALE;
    Serial.printf("[bat] raw=%lu vbat=%.2f\n", (unsigned long)raw, vbat);
    // LiPo: ~4.2V full, ~3.3V empty.
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
        if (millis() - lastBatMs >= 2000) {   // refresh battery every 2s
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
        display.drawStr(128 - display.getStrWidth(bat), 12, bat);  // top-right
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
// Each slot is an independent state machine:
//   IDLE  — on Arm(this slot, this device) → ARMED; on Fire → reply Nack.
//   ARMED — sends Armed; starts a 10s window; on Fire → FIRED; on timeout → IDLE.
//   FIRED — drives the firing line HIGH for exactly 0.5s, then reports Fired.

enum class SlotState : uint8_t { Idle, Armed, Fired };

struct Slot {
    uint32_t      id;        // slot number on the wire
    uint8_t       gpio;      // firing line (idles LOW)
    SlotState     state;
    uint32_t      action;    // action id from the Arm that armed this slot
    TimerHandle_t armTimer;  // 10s arm window (timer id = slot index)
};

static Slot s_slots[] = {
    { 1, SLOT1_FIRE_GPIO, SlotState::Idle, 0, nullptr },
    { 2, LED_GPIO, SlotState::Idle, 0, nullptr },
    { 3, LED_GPIO, SlotState::Idle, 0, nullptr },
    { 4, LED_GPIO, SlotState::Idle, 0, nullptr },
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
        struct {
            rd_Envelope env;
            float       rssi;
        } rx;
        LMLError err;
        size_t   slotIdx;   // valid when kind == ArmTimeout
    };
};

// Handoff to the dedicated firing task.
struct FireJob {
    size_t   slotIdx;
    uint32_t seq;
    uint32_t action;
};

static QueueHandle_t s_evQueue;
static QueueHandle_t s_fireQueue;

static uint8_t s_txBuf[64];

// Single best-effort transmit attempt. Returns true if LML accepted it.
static bool trySendOnce(const rd_Envelope &env) {
    size_t len = rd::encode(env, s_txBuf, sizeof(s_txBuf));
    if (len == 0) {
        Serial.println("[slave] encode failed");
        return false;
    }
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

// ── Slot transitions ─────────────────────────────────────────────────────────

// IDLE/ARMED + Arm → (re)enter ARMED: reply Armed, start the 10s window.
static void enterArmed(Slot &s, uint32_t seq, uint32_t action) {
    s.action = action;
    if (!trySendOnce(rd::make_armed(seq, s.id, DEVICE_ID, s.action))) {
        // Could not send Armed → fall back to IDLE and try Error exactly once.
        Serial.printf("[slave] slot %lu: Armed tx failed → IDLE + Error\n",
                      (unsigned long)s.id);
        s.state = SlotState::Idle;
        xTimerStop(s.armTimer, 0);
        trySendOnce(rd::make_error(seq, DEVICE_ID));
        refreshStatus();
        return;
    }
    s.state = SlotState::Armed;
    xTimerStart(s.armTimer, 0);   // (re)starts the 10s arm window
    Serial.printf("[slave] slot %lu ARMED act %lu\n",
                  (unsigned long)s.id, (unsigned long)action);
    refreshStatus();
}

static void onArm(const rd_Arm &a, uint32_t seq) {
    if (a.device_id != DEVICE_ID) return;          // not for us
    Slot *s = findSlot(a.slot);
    if (!s) return;                                 // unknown slot

    switch (s->state) {
        case SlotState::Idle:
        case SlotState::Armed:
            enterArmed(*s, seq, a.action_id);
            break;
        case SlotState::Fired:
            // Already fired — refuse to re-arm.
            trySendOnce(rd::make_disarmed(seq, s->id, DEVICE_ID, a.action_id));
            break;
    }
}

static void onFire(const rd_Fire &f, uint32_t seq) {
    if (f.device_id != DEVICE_ID) return;
    Slot *s = findSlot(f.slot);
    if (!s) return;

    if (s->state != SlotState::Armed) {
        // Not armed (IDLE or already FIRED) → Nack.
        trySendOnce(rd::make_nack(seq, DEVICE_ID));
        return;
    }

    // ARMED → FIRED. State flips here (owned by the app task); the dedicated
    // fire task performs the precisely-timed pulse and reports the outcome.
    size_t idx = (size_t)(s - s_slots);
    s->state   = SlotState::Fired;
    xTimerStop(s->armTimer, 0);
    refreshStatus();
    FireJob job = { idx, seq, f.action_id };
    xQueueSend(s_fireQueue, &job, 0);
    Serial.printf("[slave] slot %lu FIRE act %lu → pulse\n",
                  (unsigned long)s->id, (unsigned long)f.action_id);
}

// Return a single slot to IDLE: stop its arm window and drive the line LOW.
static void resetSlot(Slot &s) {
    xTimerStop(s.armTimer, 0);
    digitalWrite(s.gpio, LOW);
    s.state  = SlotState::Idle;
    s.action = 0;
}

// Master asked us to reset. slot == 0 clears every slot on this device.
static void onReset(const rd_Reset &r) {
    if (r.device_id != DEVICE_ID) return;
    if (r.slot == 0) {
        for (size_t i = 0; i < NUM_SLOTS; i++) resetSlot(s_slots[i]);
        Serial.println("[slave] RESET all slots → IDLE");
    } else {
        Slot *s = findSlot(r.slot);
        if (!s) return;
        resetSlot(*s);
        Serial.printf("[slave] RESET slot %lu → IDLE\n", (unsigned long)r.slot);
    }
    refreshStatus();
}

// ── Firing task ──────────────────────────────────────────────────────────────
// High priority, pinned: drives the line HIGH for exactly 0.5s. Timing is held
// by busy-waiting on the 64-bit microsecond clock (esp_timer_get_time) rather
// than vTaskDelay, so FreeRTOS tick granularity can't stretch the pulse.

void fireTask(void *) {
    FireJob job;
    while (xQueueReceive(s_fireQueue, &job, portMAX_DELAY) == pdTRUE) {
        Slot &s = s_slots[job.slotIdx];

        digitalWrite(s.gpio, HIGH);      // firing line: MOSFET gate HIGH → fire
        digitalWrite(LED_GPIO, HIGH);    // onboard LED: visual "firing now"
        int64_t start = esp_timer_get_time();
        while (esp_timer_get_time() - start < 1200000) { /* spin: exactly 0.75s */ }
        digitalWrite(s.gpio, LOW);
        digitalWrite(LED_GPIO, LOW);

        // Pulse complete. Report Fired; if that send fails, report Misfired.
        Serial.printf("[slave] slot %lu pulse done (%lld us)\n",
                      (unsigned long)s.id, esp_timer_get_time() - start);
        if (!trySendOnce(rd::make_fired(job.seq, s.id, DEVICE_ID, job.action))) {
            Serial.printf("[slave] slot %lu Fired tx failed → Misfired\n",
                          (unsigned long)s.id);
            trySendOnce(rd::make_misfired(job.seq, s.id, DEVICE_ID, job.action));
        }
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
                if (s.state == SlotState::Armed) {   // ignore stale ticks
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
                    case rd_Envelope_arm_tag:
                        onArm(e.payload.arm, e.seq);
                        break;
                    case rd_Envelope_fire_tag:
                        onFire(e.payload.fire, e.seq);
                        break;
                    case rd_Envelope_reset_tag:
                        onReset(e.payload.reset);
                        break;
                    default:
                        Serial.printf("[slave] ignored variant=%d\n",
                                      e.which_payload);
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

    // Firing "now" indicator LED idles LOW.
    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, LOW);

    // Firing lines must idle LOW before anything else can drive them.
    for (size_t i = 0; i < NUM_SLOTS; i++) {
        pinMode(s_slots[i].gpio, OUTPUT);
        digitalWrite(s_slots[i].gpio, LOW);
        s_slots[i].state    = SlotState::Idle;
        s_slots[i].armTimer = xTimerCreate("arm", pdMS_TO_TICKS(10000),
                                           pdFALSE, (void *)(uintptr_t)i,
                                           armTimerCb);
    }

    displayMutex = xSemaphoreCreateMutex();
    s_evQueue    = xQueueCreate(8, sizeof(AppEvent));
    s_fireQueue  = xQueueCreate(NUM_SLOTS, sizeof(FireJob));

    xTaskCreatePinnedToCore(protocolTask, "lml_s", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(pumpTask,     "pump",  4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(appTask,      "app_s", 4096, NULL, 2, NULL, 1);
    // Fire task at the highest app priority so the 0.5s pulse is not preempted.
    xTaskCreatePinnedToCore(fireTask,     "fire",  4096, NULL, 4, NULL, 1);
    xTaskCreate(displayTask, "display",   4096, NULL, 1, NULL);

    Serial.println("[slave] tasks created");
}

void loop() { vTaskDelay(portMAX_DELAY); }
