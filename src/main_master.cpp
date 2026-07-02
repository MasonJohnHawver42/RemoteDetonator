#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>
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

// User inputs (active-low, internal pull-up: GPIO → button → GND).
//   BTN_CYCLE   — cycle through the available slot/device pairs (STANDBY)
//   BTN_ADVANCE — advance the selected pair to OPERATIONAL  (STANDBY)
#define BTN_CYCLE   42
#define BTN_ADVANCE 40

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);
SX1262        radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
LML::P2PAsync mac(radio, LORA_DIO1);

// ── Available targets ────────────────────────────────────────────────────────
// Compiled into the binary: the slot/device pairs the operator may fire and the
// order they are presented in. Edit this table to match the deployment.

struct SlotDevice { uint32_t slot; uint32_t device_id; };
static const SlotDevice PAIRS[] = {
    { 1, 1 },
    { 2, 1 },
    { 3, 1 },
    { 4, 1 },
};
static constexpr size_t NUM_PAIRS = sizeof(PAIRS) / sizeof(PAIRS[0]);

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
static char dispLine1[32] = "";
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
        display.drawStr(0, 12, "[ MASTER ]");
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
    Serial.println("[master] protocolTask start");
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    int state = radio.begin(915.0, 125.0, 9, 7,
                            RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8, 1.8);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[master] radio FAILED state=%d\n", state);
        setDisplay("radio fail", "");
        vTaskDelete(NULL);
    }
    radio.setDio2AsRfSwitch(true);
    mac.init();
    Serial.println("[master] mac init OK — polling");

    while (true) {
        mac.poll();
    }
}

// ── State machine ────────────────────────────────────────────────────────────
//
// INIT        — title / error page. Any button advances to STANDBY.
// STANDBY     — BTN_CYCLE steps through PAIRS, BTN_ADVANCE arms+fires the pair.
// OPERATIONAL — runs the firing sequence for the selected pair through a small
//               sub state machine: ARM (send Arm, await Armed) → FIRE (send
//               Fire). After Fire is sent we return to STANDBY and a background
//               watcher (active in STANDBY/INIT) tracks the Fired response.

enum class MState : uint8_t { Init, Standby, Operational };
enum class OpSub  : uint8_t { Arm, Fire };

enum class EvKind : uint8_t {
    RxEnvelope,     // a decoded Envelope arrived from the peer
    LmlError,       // LML surfaced a TX error (TIMEOUT, HALT, ...)
    ArmTimeout,     // Armed response never arrived (OPERATIONAL/Arm)
    FireTimeout,    // Fired response never arrived (background watcher)
    ButtonPressed,  // debounced button press   (read .button for id)
    ButtonReleased, // debounced button release (read .button for id)
};

struct AppEvent {
    EvKind kind;
    union {
        struct {
            rd_Envelope env;
            float       rssi;
        } rx;
        LMLError err;
        uint8_t  button;    // valid when kind == Button{Pressed,Released}
    };
};

static constexpr uint32_t ARM_TIMEOUT_MS  = 10000;
static constexpr uint32_t FIRE_TIMEOUT_MS = 10000;

static QueueHandle_t s_evQueue;
static TimerHandle_t s_armTimer;    // one-shot: Armed response deadline
static TimerHandle_t s_fireTimer;   // one-shot: Fired response deadline

static MState   s_state    = MState::Init;
static OpSub    s_opSub    = OpSub::Arm;
static size_t   s_pairIdx  = 0;     // last selected pair; persists across INIT
static uint32_t s_seq      = 0;
static uint32_t s_nextAct  = 1;     // monotonic source of unique action ids
static uint32_t s_armAct   = 0;     // action id of the in-flight Arm

// Background fire-response watcher. Armed when Fire is sent; resolved in
// STANDBY/INIT by the matching Fired, a wrong response, or FireTimeout.
static bool     s_fireWaiting = false;
static uint32_t s_fireSlot    = 0;
static uint32_t s_fireDevice  = 0;
static uint32_t s_fireAct     = 0;

// STANDBY reset chord: hold BTN_CYCLE (select) and tap BTN_ADVANCE (fire).
// The tap count (applied when select is released) selects the reset scope.
static bool     s_selectDown  = false;  // BTN_CYCLE currently held
static int      s_fireTaps    = 0;      // BTN_ADVANCE taps while select held

static uint8_t  s_txBuf[64];

static uint32_t nextAction() { return s_nextAct++; }

static LMLError trySend(const rd_Envelope &env) {
    size_t len = rd::encode(env, s_txBuf, sizeof(s_txBuf));
    if (len == 0) return LMLError::BAD_LENGTH;
    return mac.queue_tx(s_txBuf, len);
}

static void armTimerCb(TimerHandle_t)  { AppEvent e; e.kind = EvKind::ArmTimeout;  xQueueSend(s_evQueue, &e, 0); }
static void fireTimerCb(TimerHandle_t) { AppEvent e; e.kind = EvKind::FireTimeout; xQueueSend(s_evQueue, &e, 0); }

// ── State entry helpers ──────────────────────────────────────────────────────

static void enterInit(const char *msg) {
    Serial.printf("[master] → INIT (%s)\n", msg);
    s_state = MState::Init;
    xTimerStop(s_armTimer, 0);
    setDisplay("Remote Det", msg);
    // The fire watcher (s_fireTimer) is intentionally left running: a pending
    // Fired response is still tracked while sitting on the INIT screen.
}

static void showPair() {
    char l2[32];
    snprintf(l2, sizeof(l2), "slot %lu dev %lu",
             (unsigned long)PAIRS[s_pairIdx].slot,
             (unsigned long)PAIRS[s_pairIdx].device_id);
    setDisplay("STANDBY", l2);
}

static void enterStandby() {
    Serial.printf("[master] → STANDBY pair=%u (slot %lu dev %lu)\n",
                  (unsigned)s_pairIdx,
                  (unsigned long)PAIRS[s_pairIdx].slot,
                  (unsigned long)PAIRS[s_pairIdx].device_id);
    s_state = MState::Standby;
    s_selectDown = false;   // no chord in progress on a fresh STANDBY
    s_fireTaps   = 0;
    showPair();
}

static void startFire();   // fwd

static void startArm() {
    const SlotDevice &p = PAIRS[s_pairIdx];
    s_opSub  = OpSub::Arm;
    s_armAct = nextAction();
    char l2[32];
    snprintf(l2, sizeof(l2), "slot %lu dev %lu",
             (unsigned long)p.slot, (unsigned long)p.device_id);
    setDisplay("Arming", l2);
    Serial.printf("[master] ARM slot %lu dev %lu act %lu\n",
                  (unsigned long)p.slot, (unsigned long)p.device_id,
                  (unsigned long)s_armAct);
    LMLError r = trySend(rd::make_arm(s_seq++, p.slot, p.device_id, s_armAct));
    if (r != LMLError::OK) {
        Serial.printf("[master] arm tx err=%d\n", (int)r);
        enterInit("arm tx error");
        return;
    }
    xTimerStart(s_armTimer, 0);
}

static void enterOperational() {
    Serial.println("[master] → OPERATIONAL");
    s_state = MState::Operational;
    // Drop any Standby leftovers in LML's TX queue so the Arm is accepted.
    mac.clear_tx();
    startArm();
}

static void startFire() {
    const SlotDevice &p = PAIRS[s_pairIdx];
    s_opSub  = OpSub::Fire;
    s_fireAct = nextAction();
    char l2[32];
    snprintf(l2, sizeof(l2), "slot %lu dev %lu",
             (unsigned long)p.slot, (unsigned long)p.device_id);
    setDisplay("Firing", l2);
    Serial.printf("[master] FIRE slot %lu dev %lu act %lu\n",
                  (unsigned long)p.slot, (unsigned long)p.device_id,
                  (unsigned long)s_fireAct);
    LMLError r = trySend(rd::make_fire(s_seq++, p.slot, p.device_id, s_fireAct));
    if (r != LMLError::OK) {
        Serial.printf("[master] fire tx err=%d\n", (int)r);
        enterInit("fire tx error");
        return;
    }
    // Arm the background watcher and return to STANDBY for the next pair.
    s_fireSlot    = p.slot;
    s_fireDevice  = p.device_id;
    s_fireWaiting = true;
    xTimerStart(s_fireTimer, 0);
    enterStandby();
}

// ── Background fire-response watcher (runs in STANDBY/INIT) ──────────────────
// Returns true when it consumed the event.

static bool fireWatcher(const AppEvent &ev) {
    if (!s_fireWaiting) return false;

    if (ev.kind == EvKind::FireTimeout) {
        s_fireWaiting = false;
        Serial.println("[master] fire response timeout");
        enterInit("fire timeout");
        return true;
    }
    if (ev.kind == EvKind::RxEnvelope) {
        const rd_Envelope &e = ev.rx.env;
        if (e.which_payload == rd_Envelope_fired_tag) {
            const rd_Fired &f = e.payload.fired;
            if (f.slot == s_fireSlot && f.device_id == s_fireDevice &&
                f.action_id == s_fireAct) {
                s_fireWaiting = false;
                xTimerStop(s_fireTimer, 0);
                Serial.println("[master] Fired OK (correct) — no action");
                return true;   // correct → do nothing for now
            }
            s_fireWaiting = false;
            xTimerStop(s_fireTimer, 0);
            enterInit("bad fire resp");
            return true;
        }
        if (e.which_payload == rd_Envelope_misfired_tag) {
            s_fireWaiting = false;
            xTimerStop(s_fireTimer, 0);
            enterInit("MISFIRE");
            return true;
        }
    }
    return false;
}

// ── Reset chord (STANDBY) ────────────────────────────────────────────────────
// Fire-and-forget: send the Reset(s) and drop back to INIT with a summary.

static void sendReset(uint32_t slot, uint32_t device_id) {
    LMLError r = trySend(rd::make_reset(s_seq++, slot, device_id, nextAction()));
    if (r != LMLError::OK)
        Serial.printf("[master] reset tx err=%d (slot %lu dev %lu)\n",
                      (int)r, (unsigned long)slot, (unsigned long)device_id);
}

// taps 1 → current slot, 2 → whole current device, 3+ → every device in PAIRS.
static void doReset(int taps) {
    const SlotDevice &p = PAIRS[s_pairIdx];
    char msg[32];
    if (taps <= 1) {
        sendReset(p.slot, p.device_id);
        snprintf(msg, sizeof(msg), "reset slot %lu", (unsigned long)p.slot);
    } else if (taps == 2) {
        sendReset(0, p.device_id);              // slot 0 = all slots on device
        snprintf(msg, sizeof(msg), "reset dev %lu", (unsigned long)p.device_id);
    } else {
        for (size_t i = 0; i < NUM_PAIRS; i++) {   // each unique device once
            uint32_t d = PAIRS[i].device_id;
            bool seen = false;
            for (size_t j = 0; j < i; j++)
                if (PAIRS[j].device_id == d) { seen = true; break; }
            if (!seen) sendReset(0, d);
        }
        snprintf(msg, sizeof(msg), "reset all devs");
    }
    Serial.printf("[master] %s (taps=%d)\n", msg, taps);
    enterInit(msg);
}

// ── Per-state handlers ───────────────────────────────────────────────────────

static void handleInit(const AppEvent &ev) {
    if (fireWatcher(ev)) return;

    if (ev.kind == EvKind::ButtonReleased) {
        // A release leaves the title/error page. Consume everything else queued
        // (further presses/releases) so it doesn't act on the STANDBY screen;
        // route non-button events through the watcher so a Fired isn't lost.
        AppEvent tmp;
        while (xQueueReceive(s_evQueue, &tmp, 0) == pdTRUE) {
            if (tmp.kind == EvKind::ButtonPressed || tmp.kind == EvKind::ButtonReleased)
                continue;
            fireWatcher(tmp);
        }
        enterStandby();   // s_pairIdx holds the last selection (0 = first pair)
    }
}

static void handleStandby(const AppEvent &ev) {
    if (fireWatcher(ev)) return;

    switch (ev.kind) {
        case EvKind::ButtonPressed:
            if (ev.button == BTN_CYCLE) {
                s_selectDown = true;    // begin a (possible) reset chord
                s_fireTaps   = 0;
            } else if (ev.button == BTN_ADVANCE && s_selectDown) {
                if (s_fireTaps < 3) s_fireTaps++;   // count taps while select held
            }
            break;
        case EvKind::ButtonReleased:
            if (ev.button == BTN_CYCLE) {
                bool wasChord = s_selectDown && s_fireTaps > 0;
                s_selectDown = false;
                if (wasChord) {
                    doReset(s_fireTaps);            // → INIT with a summary
                } else {
                    s_pairIdx = (s_pairIdx + 1) % NUM_PAIRS;   // plain cycle
                    showPair();
                }
                s_fireTaps = 0;
            } else if (ev.button == BTN_ADVANCE) {
                if (!s_selectDown) enterOperational();   // fire on a plain release
            }
            break;
        default:
            break;   // stale timer ticks, stray Rx, link errors
    }
}

static void handleOperational(const AppEvent &ev) {
    // OPERATIONAL only ever waits in the ARM sub-state — FIRE sends and
    // immediately returns to STANDBY, so the FIRE response is the watcher's job.
    if (s_opSub != OpSub::Arm) return;

    const SlotDevice &p = PAIRS[s_pairIdx];
    switch (ev.kind) {
        case EvKind::RxEnvelope: {
            const rd_Envelope &e = ev.rx.env;
            if (e.which_payload == rd_Envelope_armed_tag) {
                const rd_Armed &a = e.payload.armed;
                if (a.slot == p.slot && a.device_id == p.device_id &&
                    a.action_id == s_armAct) {
                    Serial.println("[master] Armed OK → FIRE");
                    xTimerStop(s_armTimer, 0);
                    startFire();
                } else {
                    xTimerStop(s_armTimer, 0);
                    enterInit("bad arm resp");
                }
            } else if (e.which_payload == rd_Envelope_disarmed_tag) {
                xTimerStop(s_armTimer, 0);
                enterInit("already fired");
            } else {
                xTimerStop(s_armTimer, 0);
                enterInit("bad arm resp");
            }
            break;
        }
        case EvKind::LmlError:
            Serial.printf("[master] arm link err=%d\n", (int)ev.err);
            xTimerStop(s_armTimer, 0);
            enterInit("arm link error");
            break;
        case EvKind::ArmTimeout:
            enterInit("arm timeout");
            break;
        default:
            break;   // ignore buttons / stale fire ticks while arming
    }
}

// ── Buttons ──────────────────────────────────────────────────────────────────
// Debounced, active-low GPIO buttons. Posts ButtonPressed/ButtonReleased
// carrying the button id (its GPIO number) so the FSM stays event-driven.

struct Button {
    uint8_t       id;
    uint8_t       pin;
    QueueHandle_t queue;
    bool          stable;      // last debounced level (true = released/high)
    bool          lastRaw;
    uint32_t      lastEdgeMs;
};

static constexpr uint32_t BTN_DEBOUNCE_MS = 30;

static void buttonInit(Button &b, uint8_t id, uint8_t pin, QueueHandle_t q) {
    b.id         = id;
    b.pin        = pin;
    b.queue      = q;
    pinMode(pin, INPUT_PULLUP);
    delay(5);                                   // let the pull-up settle
    bool raw      = (digitalRead(pin) != 0);    // seed from the real level so a
    b.stable      = raw;                         //   pin that idles low at boot
    b.lastRaw     = raw;                         //   doesn't read as a press
    b.lastEdgeMs  = millis();
}

static void buttonPoll(Button &b) {
    bool     raw = (digitalRead(b.pin) != 0);   // true = released, false = pressed
    uint32_t now = millis();

    if (raw != b.lastRaw) {
        b.lastRaw    = raw;
        b.lastEdgeMs = now;
        return;
    }
    if (now - b.lastEdgeMs < BTN_DEBOUNCE_MS) return;

    if (raw != b.stable) {
        b.stable = raw;
        AppEvent ev;
        ev.kind   = b.stable ? EvKind::ButtonReleased : EvKind::ButtonPressed;
        ev.button = b.id;
        Serial.printf("[btn] gpio %u %s\n", b.id, b.stable ? "released" : "PRESSED");
        xQueueSend(b.queue, &ev, 0);
    }
}

static Button s_buttons[2];

void buttonTask(void *) {
    uint32_t lastDump = 0;
    while (true) {
        buttonPoll(s_buttons[0]);
        buttonPoll(s_buttons[1]);
        // DEBUG: 1 Hz raw level of the two button pins (1 = released, 0 = pressed).
        uint32_t now = millis();
        if (now - lastDump >= 1000) {
            lastDump = now;
            Serial.printf("[btn] raw gpio%u=%d gpio%u=%d\n",
                          s_buttons[0].pin, digitalRead(s_buttons[0].pin),
                          s_buttons[1].pin, digitalRead(s_buttons[1].pin));
        }
        vTaskDelay(pdMS_TO_TICKS(5));
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
                Serial.printf("[master] pump: decode failed len=%u\n", lml.msg.len);
                appEv.kind = EvKind::LmlError;
                appEv.err  = LMLError::BAD_LENGTH;
            }
        }
        xQueueSend(s_evQueue, &appEv, portMAX_DELAY);
    }
}

// ── App: state machine driver ───────────────────────────────────────────────

void appTask(void *) {
    Serial.println("[master] appTask start — waiting 2s for radio");
    vTaskDelay(pdMS_TO_TICKS(2000));

    enterInit("press a button");

    AppEvent ev;
    while (true) {
        if (xQueueReceive(s_evQueue, &ev, portMAX_DELAY) != pdTRUE) continue;
        switch (s_state) {
            case MState::Init:        handleInit(ev);        break;
            case MState::Standby:     handleStandby(ev);     break;
            case MState::Operational: handleOperational(ev); break;
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("[master] boot");

    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, LOW);
    delay(100);

    displayMutex = xSemaphoreCreateMutex();

    s_evQueue   = xQueueCreate(8, sizeof(AppEvent));
    s_armTimer  = xTimerCreate("arm",  pdMS_TO_TICKS(ARM_TIMEOUT_MS),
                               pdFALSE, NULL, armTimerCb);
    s_fireTimer = xTimerCreate("fire", pdMS_TO_TICKS(FIRE_TIMEOUT_MS),
                               pdFALSE, NULL, fireTimerCb);

    buttonInit(s_buttons[0], BTN_CYCLE,   BTN_CYCLE,   s_evQueue);
    buttonInit(s_buttons[1], BTN_ADVANCE, BTN_ADVANCE, s_evQueue);

    xTaskCreatePinnedToCore(protocolTask, "lml_m", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(pumpTask,     "pump",  4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(appTask,      "app_m", 4096, NULL, 2, NULL, 1);
    xTaskCreate(displayTask, "display",   4096, NULL, 1, NULL);
    xTaskCreate(buttonTask,  "button",    2048, NULL, 2, NULL);

    Serial.println("[master] tasks created");
}

void loop() { vTaskDelay(portMAX_DELAY); }
