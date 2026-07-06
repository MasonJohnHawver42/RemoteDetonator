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

// [B] = BTN_CYCLE (cycle / select)   [R] = BTN_ADVANCE (fire / advance)
#define BTN_CYCLE   42
#define BTN_ADVANCE 40

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);
SX1262        radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
LML::P2PAsync mac(radio, LORA_DIO1);

// ── Available targets ────────────────────────────────────────────────────────

struct SlotDevice { uint32_t slot; uint32_t device_id; };
static const SlotDevice PAIRS[] = {
    { 1, 1 },
    { 2, 1 },
    { 3, 1 },
    { 4, 1 },
};
static constexpr size_t NUM_PAIRS = sizeof(PAIRS) / sizeof(PAIRS[0]);

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
//
// 128×64 OLED layout:
//   y=10  [ MASTER ]           NN%   ncenB08  header (always)
//   y=22  state + sub-state          ncenB08  row 1
//   y=34  supporting info            ncenB08  row 2
//   y=46  button hints               5x7      row 3

static SemaphoreHandle_t displayMutex;
static char dispLine1[32] = "";   // state + sub-state
static char dispLine2[32] = "";   // info
static char dispLine3[32] = "";   // button hints

static void setDisplay(const char *l1, const char *l2, const char *l3 = "") {
    xSemaphoreTake(displayMutex, portMAX_DELAY);
    strncpy(dispLine1, l1, sizeof(dispLine1) - 1); dispLine1[sizeof(dispLine1)-1] = '\0';
    strncpy(dispLine2, l2, sizeof(dispLine2) - 1); dispLine2[sizeof(dispLine2)-1] = '\0';
    strncpy(dispLine3, l3, sizeof(dispLine3) - 1); dispLine3[sizeof(dispLine3)-1] = '\0';
    xSemaphoreGive(displayMutex);
}

void displayTask(void *) {
    display.begin();
    char l1[32], l2[32], l3[32];
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
        strncpy(l3, dispLine3, sizeof(l3));
        xSemaphoreGive(displayMutex);

        display.clearBuffer();

        display.setFont(u8g2_font_ncenB08_tr);
        display.drawStr(0, 10, "[ MASTER ]");
        char bat[8];
        snprintf(bat, sizeof(bat), "%d%%", batPct);
        display.drawStr(128 - display.getStrWidth(bat), 10, bat);
        display.drawStr(0, 26, l1);
        display.drawStr(0, 38, l2);

        display.setFont(u8g2_font_5x7_tf);
        display.drawStr(0, 60, l3);

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
        setDisplay("INIT", "radio fail");
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

enum class MState    : uint8_t { Init, Standby, Operational, Maintenance };
enum class StandbySub: uint8_t { Idle, ResetPrep, OperationalPrep };
enum class OpSub     : uint8_t { ArmDevice, FireDevice, ShowResult };

enum class EvKind : uint8_t {
    RxEnvelope,
    LmlError,
    ArmTimeout,
    FireTimeout,
    ButtonPressed,
    ButtonReleased,
};

struct AppEvent {
    EvKind kind;
    union {
        struct { rd_Envelope env; float rssi; } rx;
        LMLError err;
        uint8_t  button;
    };
};

static constexpr uint32_t ARM_TIMEOUT_MS = 10000;

static constexpr uint32_t PULSE_DUR_DEFAULT_MS = 1200;
static constexpr uint32_t PULSE_DUR_MIN_MS     =  100;
static constexpr uint32_t PULSE_DUR_MAX_MS     = 5000;
static constexpr uint32_t PULSE_DUR_STEP_MS    =  100;

static QueueHandle_t s_evQueue;
static TimerHandle_t s_armTimer;
static TimerHandle_t s_fireTimer;

// ── FSM state ────────────────────────────────────────────────────────────────

static MState     s_state      = MState::Init;
static StandbySub s_standbySub = StandbySub::Idle;
static OpSub      s_opSub      = OpSub::ArmDevice;

static size_t   s_pairIdx = 0;
static uint32_t s_seq     = 0;
static uint32_t s_nextAct = 1;
static uint32_t s_armAct  = 0;
static uint32_t s_fireAct = 0;

static bool     s_selected[NUM_PAIRS];

// Per-pair default fire pulse durations (ms). Edit to bake in slot-specific times.
// Order matches PAIRS[]: { slot, device_id } → { {1,1}, {2,1}, {3,1}, {4,1} }
static uint32_t s_pulseDurMs[NUM_PAIRS] = {
    1200,   // d1 s1
    1400,   // d1 s2
    1200,   // d1 s3
    1200,   // d1 s4
};
static uint32_t s_lastBtnMs = 0;   // millis() of the most recent button press (Standby entry detection)
static uint8_t  s_lastBtn   = 0;   // which button ID was pressed last

static uint8_t  s_maintBtnHeld   = 0;     // buttons currently held in Maintenance
static bool     s_maintExitArmed = false;  // true once both buttons have been pressed

// ── Per-device arm+fire results ──────────────────────────────────────────────

struct OpDevice {
    uint32_t device_id;
    uint32_t slots[NUM_PAIRS];
    uint8_t  slot_count;
    uint32_t armed_slots[NUM_PAIRS];
    uint8_t  armed_count;
    bool     arm_lml_error;
    bool     arm_timeout;
    bool     fire_lml_error;
    bool     fire_timeout;
    bool     fire_success;
};

static OpDevice s_opDevices[NUM_PAIRS];
static size_t   s_opDevCount = 0;
static size_t   s_opDevIdx   = 0;

static uint8_t s_txBuf[128];

// ── Helpers ──────────────────────────────────────────────────────────────────

static uint32_t nextAction() { return s_nextAct++; }

static LMLError trySend(const rd_Envelope &env) {
    size_t len = rd::encode(env, s_txBuf, sizeof(s_txBuf));
    if (len == 0) return LMLError::BAD_LENGTH;
    return mac.queue_tx(s_txBuf, len);
}

static void armTimerCb(TimerHandle_t)  {
    AppEvent e; e.kind = EvKind::ArmTimeout;  xQueueSend(s_evQueue, &e, 0);
}
static void fireTimerCb(TimerHandle_t) {
    AppEvent e; e.kind = EvKind::FireTimeout; xQueueSend(s_evQueue, &e, 0);
}

// "D1 s2 s3 D2 s1" — slots grouped under their device, uppercase D.
static void buildSelectionStr(char *out, size_t cap) {
    int n = 0;
    out[0] = '\0';
    for (size_t i = 0; i < NUM_PAIRS; i++) {
        if (!s_selected[i]) continue;
        uint32_t dev = PAIRS[i].device_id;
        // skip if we already printed this device
        bool seen = false;
        for (size_t j = 0; j < i; j++)
            if (s_selected[j] && PAIRS[j].device_id == dev) { seen = true; break; }
        if (seen) continue;
        if (n > 0 && n < (int)cap - 2) out[n++] = ' ';
        n += snprintf(out + n, cap - n, "D%lu", (unsigned long)dev);
        for (size_t k = i; k < NUM_PAIRS && n < (int)cap - 4; k++) {
            if (!s_selected[k] || PAIRS[k].device_id != dev) continue;
            n += snprintf(out + n, cap - n, " s%lu", (unsigned long)PAIRS[k].slot);
        }
    }
    if (n == 0) strncpy(out, "none", cap);
}

// "D1 s1 s2 s3" for a specific device + slot list.
static void buildDevSlotStr(uint32_t device_id,
                             const uint32_t *slots, uint8_t count,
                             char *out, size_t cap) {
    int n = snprintf(out, cap, "D%lu", (unsigned long)device_id);
    for (uint8_t i = 0; i < count && n < (int)cap - 5; i++)
        n += snprintf(out + n, cap - n, " s%lu", (unsigned long)slots[i]);
}

static uint32_t pulseDurForSlot(uint32_t device_id, uint32_t slot_id) {
    for (size_t i = 0; i < NUM_PAIRS; i++)
        if (PAIRS[i].device_id == device_id && PAIRS[i].slot == slot_id)
            return s_pulseDurMs[i];
    return PULSE_DUR_DEFAULT_MS;
}

// Priority-ordered result message.
static void buildResultMsg(char *buf, size_t len) {
    for (size_t d = 0; d < s_opDevCount; d++) {
        if (s_opDevices[d].arm_lml_error || s_opDevices[d].fire_lml_error) {
            snprintf(buf, len, "link err d%lu", (unsigned long)s_opDevices[d].device_id);
            return;
        }
    }
    for (size_t d = 0; d < s_opDevCount; d++) {
        if (s_opDevices[d].arm_timeout) {
            snprintf(buf, len, "arm timeout d%lu", (unsigned long)s_opDevices[d].device_id);
            return;
        }
    }
    for (size_t d = 0; d < s_opDevCount; d++) {
        if (s_opDevices[d].fire_timeout) {
            snprintf(buf, len, "fire timeout d%lu", (unsigned long)s_opDevices[d].device_id);
            return;
        }
    }
    bool all_ok = true;
    for (size_t d = 0; d < s_opDevCount; d++)
        if (!s_opDevices[d].fire_success) { all_ok = false; break; }
    snprintf(buf, len, all_ok ? "Mission Success" : "partial failure");
}

// ── Screen helpers ───────────────────────────────────────────────────────────

static void enterInit(const char *msg) {
    Serial.printf("[master] → INIT (%s)\n", msg);
    s_state = MState::Init;
    xTimerStop(s_armTimer,  0);
    xTimerStop(s_fireTimer, 0);
    setDisplay("INIT", msg, "[B]/[R] advance");
}

static void enterInitSub(const char *sub, const char *info) {
    char l1[32];
    snprintf(l1, sizeof(l1), "INIT - %s", sub);
    Serial.printf("[master] → INIT (%s: %s)\n", sub, info);
    s_state = MState::Init;
    xTimerStop(s_armTimer,  0);
    xTimerStop(s_fireTimer, 0);
    setDisplay(l1, info, "[B]/[R] advance");
}

// ── Maintenance state ────────────────────────────────────────────────────────

static void showMaintenance() {
    char l2[32];
    snprintf(l2, sizeof(l2), "D%lu s%lu = %lu ms",
             (unsigned long)PAIRS[s_pairIdx].device_id,
             (unsigned long)PAIRS[s_pairIdx].slot,
             (unsigned long)s_pulseDurMs[s_pairIdx]);
    setDisplay("MAINTENANCE", l2, "[B]- [R]+ [B+R]exit");
}

static void enterMaintenance() {
    Serial.printf("[master] → MAINTENANCE pair=%u dur=%lu ms\n",
                  (unsigned)s_pairIdx, (unsigned long)s_pulseDurMs[s_pairIdx]);
    s_state            = MState::Maintenance;
    s_maintBtnHeld     = 0;
    s_maintExitArmed   = false;
    showMaintenance();
}

static void handleMaintenance(const AppEvent &ev) {
    uint8_t bit = (ev.button == BTN_CYCLE) ? 0x01u : 0x02u;

    if (ev.kind == EvKind::ButtonPressed) {
        s_maintBtnHeld |= bit;
        if (s_maintBtnHeld == 0x03) {
            s_maintExitArmed = true;
            return;   // both held — wait for release, no duration change
        }
        if (s_maintExitArmed) return;   // don't act while exit is armed

        if (ev.button == BTN_CYCLE) {
            if (s_pulseDurMs[s_pairIdx] > PULSE_DUR_MIN_MS)
                s_pulseDurMs[s_pairIdx] -= PULSE_DUR_STEP_MS;
        } else {
            if (s_pulseDurMs[s_pairIdx] < PULSE_DUR_MAX_MS)
                s_pulseDurMs[s_pairIdx] += PULSE_DUR_STEP_MS;
        }
        showMaintenance();

    } else if (ev.kind == EvKind::ButtonReleased) {
        s_maintBtnHeld &= ~bit;
        if (s_maintExitArmed && s_maintBtnHeld == 0) {
            char msg[32];
            snprintf(msg, sizeof(msg), "D%lu s%lu = %lu ms",
                     (unsigned long)PAIRS[s_pairIdx].device_id,
                     (unsigned long)PAIRS[s_pairIdx].slot,
                     (unsigned long)s_pulseDurMs[s_pairIdx]);
            enterInit(msg);
        }
    }
}

// ── Standby screens ──────────────────────────────────────────────────────────

static void showPair() {
    char l2[32];
    snprintf(l2, sizeof(l2), "device %lu slot %lu",
             (unsigned long)PAIRS[s_pairIdx].device_id,
             (unsigned long)PAIRS[s_pairIdx].slot);
    setDisplay("STANDBY",
               l2,
               "[B]cyc [R]->fire [B+R]rst");
}

// Reset scope preview: what would be reset given the current tap count.
static void showResetPrep() {
    bool any = false;
    for (size_t i = 0; i < NUM_PAIRS; i++)
        if (s_selected[i]) { any = true; break; }
    if (!any) {
        setDisplay("STANDBY - Reset", "", "[B]rel=cyc [R]sel");
        return;
    }
    char sel[32];
    buildSelectionStr(sel, sizeof(sel));
    setDisplay("STANDBY - Reset Sel", sel, "[B]rel=rst [R]add");
}

static void showSelectionDisplay() {
    char sel[32];
    buildSelectionStr(sel, sizeof(sel));
    setDisplay("STANDBY - Fire Sel", sel, "[B]add [R]rel->fire");
}

static void enterStandby() {
    s_pairIdx    = 0;
    s_state      = MState::Standby;
    s_standbySub = StandbySub::Idle;
    memset(s_selected, 0, sizeof(s_selected));
    Serial.printf("[master] → STANDBY pair=%u\n", (unsigned)s_pairIdx);
    showPair();
}

// ── Reset chord ──────────────────────────────────────────────────────────────
// Sends one Reset per device containing all selected slots for that device.

static void doReset() {
    char sel[32];
    buildSelectionStr(sel, sizeof(sel));

    for (size_t i = 0; i < NUM_PAIRS; i++) {
        if (!s_selected[i]) continue;
        uint32_t dev = PAIRS[i].device_id;
        bool seen = false;
        for (size_t j = 0; j < i; j++)
            if (s_selected[j] && PAIRS[j].device_id == dev) { seen = true; break; }
        if (seen) continue;

        uint32_t  slots[NUM_PAIRS];
        pb_size_t count = 0;
        for (size_t k = i; k < NUM_PAIRS; k++) {
            if (s_selected[k] && PAIRS[k].device_id == dev)
                slots[count++] = PAIRS[k].slot;
        }
        LMLError r = trySend(rd::make_reset(s_seq++, dev, nextAction(), slots, count));
        if (r != LMLError::OK)
            Serial.printf("[master] reset tx err=%d\n", (int)r);
    }

    char msg[32];
    snprintf(msg, sizeof(msg), "reset %s", sel);
    Serial.printf("[master] %s\n", msg);
    enterInit(msg);
}

// ── Operational: device list from selection ──────────────────────────────────

static void buildOpDevices() {
    s_opDevCount = 0;
    memset(s_opDevices, 0, sizeof(s_opDevices));
    for (size_t i = 0; i < NUM_PAIRS; i++) {
        if (!s_selected[i]) continue;
        uint32_t dev = PAIRS[i].device_id;
        size_t d;
        for (d = 0; d < s_opDevCount; d++)
            if (s_opDevices[d].device_id == dev) break;
        if (d == s_opDevCount) {
            s_opDevices[d].device_id = dev;
            s_opDevCount++;
        }
        s_opDevices[d].slots[s_opDevices[d].slot_count++] = PAIRS[i].slot;
    }
}

// fire_timeout_ms sent to slaves in MultiArm: time until the master will reach
// the fire phase for that device = arm exchanges for all devices + 7.5 s buffer
// + sum of all requested slot durations across all devices.
static uint32_t calcFireTimeout() {
    uint32_t tmo = 7500 + (uint32_t)s_opDevCount * ARM_TIMEOUT_MS;
    for (size_t d = 0; d < s_opDevCount; d++)
        for (uint8_t si = 0; si < s_opDevices[d].slot_count; si++)
            tmo += pulseDurForSlot(s_opDevices[d].device_id, s_opDevices[d].slots[si]);
    return tmo;
}

// How long to wait for a FireResponse from one device: 7.5 s + sum of that
// device's armed slot durations (the slave fires them back-to-back).
static uint32_t calcDeviceFireTimeout(const OpDevice &d) {
    uint32_t tmo = 7500;
    for (uint8_t i = 0; i < d.armed_count; i++)
        tmo += pulseDurForSlot(d.device_id, d.armed_slots[i]);
    return tmo;
}

// ── Operational sub-state transitions ────────────────────────────────────────

static void advanceArm();
static void advanceFire();
static void startFirePhase();
static void showOperationalResult();

static void startArmDevice() {
    OpDevice &d = s_opDevices[s_opDevIdx];
    char l1[32], l2[32];
    snprintf(l1, sizeof(l1), "OPERATE - Arming %u/%u",
             (unsigned)(s_opDevIdx + 1), (unsigned)s_opDevCount);
    buildDevSlotStr(d.device_id, d.slots, d.slot_count, l2, sizeof(l2));
    setDisplay(l1, l2);

    s_armAct = nextAction();
    uint32_t tmo = calcFireTimeout();
    Serial.printf("[master] ARM dev %lu slots=%u act %lu tmo=%lums\n",
                  (unsigned long)d.device_id, d.slot_count,
                  (unsigned long)s_armAct, (unsigned long)tmo);

    LMLError r = trySend(rd::make_multi_arm(s_seq++, d.device_id, s_armAct,
                                             d.slots, d.slot_count, tmo));
    if (r != LMLError::OK) {
        Serial.printf("[master] arm tx err=%d → skip device\n", (int)r);
        d.arm_lml_error = true;
        advanceArm();
        return;
    }
    xTimerStart(s_armTimer, 0);
}

static void advanceArm() {
    xTimerStop(s_armTimer, 0);
    s_opDevIdx++;
    if (s_opDevIdx < s_opDevCount)
        startArmDevice();
    else
        startFirePhase();
}

static void startFireDevice() {
    OpDevice &d = s_opDevices[s_opDevIdx];
    char l1[32], l2[32];
    snprintf(l1, sizeof(l1), "OPERATE - Firing %u/%u",
             (unsigned)(s_opDevIdx + 1), (unsigned)s_opDevCount);
    buildDevSlotStr(d.device_id, d.armed_slots, d.armed_count, l2, sizeof(l2));
    setDisplay(l1, l2);

    s_fireAct = nextAction();
    uint32_t durations[NUM_PAIRS];
    for (uint8_t i = 0; i < d.armed_count; i++)
        durations[i] = pulseDurForSlot(d.device_id, d.armed_slots[i]);
    Serial.printf("[master] FIRE dev %lu armed=%u act %lu\n",
                  (unsigned long)d.device_id, d.armed_count, (unsigned long)s_fireAct);

    uint32_t fireTmo = calcDeviceFireTimeout(d);
    LMLError r = trySend(rd::make_multi_fire(s_seq++, d.device_id, s_fireAct,
                                              d.armed_slots, d.armed_count, durations));
    if (r != LMLError::OK) {
        Serial.printf("[master] fire tx err=%d → skip device\n", (int)r);
        d.fire_lml_error = true;
        advanceFire();
        return;
    }
    Serial.printf("[master] fire tmo=%lu ms\n", (unsigned long)fireTmo);
    xTimerChangePeriod(s_fireTimer, pdMS_TO_TICKS(fireTmo), 0);
}

// Called after FireResponse or FireTimeout — advances PAST the current device.
static void advanceFire() {
    xTimerStop(s_fireTimer, 0);
    s_opDevIdx++;
    while (s_opDevIdx < s_opDevCount && s_opDevices[s_opDevIdx].armed_count == 0)
        s_opDevIdx++;
    if (s_opDevIdx < s_opDevCount)
        startFireDevice();
    else
        showOperationalResult();
}

// Called once at the start of the fire phase (resets s_opDevIdx to 0).
static void startFirePhase() {
    s_opSub    = OpSub::FireDevice;
    s_opDevIdx = 0;
    while (s_opDevIdx < s_opDevCount && s_opDevices[s_opDevIdx].armed_count == 0)
        s_opDevIdx++;
    if (s_opDevIdx >= s_opDevCount)
        showOperationalResult();
    else
        startFireDevice();
}

// Show ShowResult only if at least one slave confirmed a fire.
// If nothing fired, go directly to Init — with a special two-line screen
// when the cause is that no slots were armed (i.e. they need to be reset).
static void showOperationalResult() {
    bool any_fired = false;
    for (size_t d = 0; d < s_opDevCount; d++)
        if (s_opDevices[d].fire_success) { any_fired = true; break; }

    if (any_fired) {
        char sel[32];
        buildSelectionStr(sel, sizeof(sel));
        setDisplay("OPERATE - Complete", sel, "[B]/[R] dismiss");
        s_opSub = OpSub::ShowResult;
        Serial.println("[master] → ShowResult");
        return;
    }

    // If a device responded but armed nothing (slots already fired / rejected),
    // show the specific two-line "Slots not armed" INIT screen.
    bool not_armed = false;
    for (size_t d = 0; d < s_opDevCount; d++) {
        const OpDevice &od = s_opDevices[d];
        if (od.armed_count == 0 && !od.arm_lml_error && !od.arm_timeout) {
            not_armed = true; break;
        }
    }
    if (not_armed) {
        char info[32] = "Reset: ";
        int n = 0;
        for (size_t d = 0; d < s_opDevCount; d++) {
            const OpDevice &od = s_opDevices[d];
            if (od.armed_count > 0 || od.arm_lml_error || od.arm_timeout) continue;
            if (n > 0 && n < (int)sizeof(info) - 2) info[n++] = ' ';
            n += snprintf(info + n, sizeof(info) - n, "D%lu", (unsigned long)od.device_id);
            for (uint8_t si = 0; si < od.slot_count && n < (int)sizeof(info) - 4; si++)
                n += snprintf(info + n, sizeof(info) - n, " s%lu", (unsigned long)od.slots[si]);
        }
        enterInitSub("Slots not armed", info);
        return;
    }

    char msg[32];
    buildResultMsg(msg, sizeof(msg));
    Serial.printf("[master] nothing confirmed fired → Init: %s\n", msg);
    enterInit(msg);
}

static void enterOperational() {
    Serial.println("[master] → OPERATIONAL");
    s_state  = MState::Operational;
    s_opSub  = OpSub::ArmDevice;
    mac.clear_tx();
    buildOpDevices();
    s_opDevIdx = 0;
    startArmDevice();
}

// ── Per-state handlers ───────────────────────────────────────────────────────

static void handleInit(const AppEvent &ev) {
    if (ev.kind == EvKind::ButtonReleased) {
        AppEvent tmp;
        while (xQueueReceive(s_evQueue, &tmp, 0) == pdTRUE) {
            if (tmp.kind != EvKind::ButtonPressed &&
                tmp.kind != EvKind::ButtonReleased)
                xQueueSend(s_evQueue, &tmp, 0);
        }
        enterStandby();
    }
}

static void handleStandby(const AppEvent &ev) {
    switch (s_standbySub) {

        case StandbySub::Idle:
            if (ev.kind == EvKind::ButtonPressed) {
                s_lastBtnMs = millis();
                s_lastBtn   = ev.button;
                if (ev.button == BTN_ADVANCE) {
                    memset(s_selected, 0, sizeof(s_selected));
                    s_selected[s_pairIdx] = true;
                    s_standbySub = StandbySub::OperationalPrep;
                    showSelectionDisplay();
                } else if (ev.button == BTN_CYCLE) {
                    memset(s_selected, 0, sizeof(s_selected));
                    s_standbySub = StandbySub::ResetPrep;
                    // display stays on current pair until [R] selects something
                }
            } else if (ev.kind == EvKind::ButtonReleased) {
                if (ev.button == BTN_CYCLE) {
                    s_pairIdx = (s_pairIdx + 1) % NUM_PAIRS;
                    showPair();
                }
            }
            break;

        case StandbySub::ResetPrep:
            if (ev.kind == EvKind::ButtonPressed) {
                if (ev.button == BTN_ADVANCE) {
                    // [R] within 50 ms of the [B] that entered this state → Maintenance
                    if (s_lastBtn == BTN_CYCLE && (millis() - s_lastBtnMs) < 50) {
                        enterMaintenance();
                        return;
                    }
                    s_selected[s_pairIdx] = true;
                    s_pairIdx = (s_pairIdx + 1) % NUM_PAIRS;
                    showResetPrep();
                }
            } else if (ev.kind == EvKind::ButtonReleased) {
                if (ev.button == BTN_CYCLE) {
                    bool any = false;
                    for (size_t i = 0; i < NUM_PAIRS; i++)
                        if (s_selected[i]) { any = true; break; }
                    if (any) {
                        doReset();
                    } else {
                        s_pairIdx = (s_pairIdx + 1) % NUM_PAIRS;
                        showPair();
                        s_standbySub = StandbySub::Idle;
                    }
                }
            }
            break;

        case StandbySub::OperationalPrep:
            if (ev.kind == EvKind::ButtonPressed) {
                if (ev.button == BTN_CYCLE) {
                    // [B] within 50 ms of the [R] that entered this state → Maintenance
                    if (s_lastBtn == BTN_ADVANCE && (millis() - s_lastBtnMs) < 50) {
                        enterMaintenance();
                        return;
                    }
                    s_pairIdx = (s_pairIdx + 1) % NUM_PAIRS;
                    s_selected[s_pairIdx] = true;
                    showSelectionDisplay();
                }
            } else if (ev.kind == EvKind::ButtonReleased) {
                if (ev.button == BTN_ADVANCE)
                    enterOperational();
            }
            break;
    }
}

static void handleOperational(const AppEvent &ev) {
    switch (s_opSub) {

        case OpSub::ArmDevice:
            switch (ev.kind) {
                case EvKind::RxEnvelope: {
                    const rd_Envelope &e = ev.rx.env;
                    if (e.which_payload != rd_Envelope_arm_response_tag) break;
                    const rd_ArmResponse &ar = e.payload.arm_response;
                    OpDevice &d = s_opDevices[s_opDevIdx];
                    if (ar.device_id != d.device_id || ar.action_id != s_armAct) break;
                    for (pb_size_t i = 0; i < ar.slots_armed_count && d.armed_count < NUM_PAIRS; i++)
                        d.armed_slots[d.armed_count++] = ar.slots_armed[i];
                    Serial.printf("[master] ArmResponse dev %lu armed=%u fired=%u failed=%u\n",
                                  (unsigned long)d.device_id, ar.slots_armed_count,
                                  ar.slots_already_fired_count, ar.slots_failed_count);
                    advanceArm();
                    break;
                }
                case EvKind::LmlError:
                    s_opDevices[s_opDevIdx].arm_lml_error = true;
                    advanceArm();
                    break;
                case EvKind::ArmTimeout:
                    s_opDevices[s_opDevIdx].arm_timeout = true;
                    advanceArm();
                    break;
                default: break;
            }
            break;

        case OpSub::FireDevice:
            switch (ev.kind) {
                case EvKind::RxEnvelope: {
                    const rd_Envelope &e = ev.rx.env;
                    if (e.which_payload != rd_Envelope_fire_response_tag) break;
                    const rd_FireResponse &fr = e.payload.fire_response;
                    OpDevice &d = s_opDevices[s_opDevIdx];
                    if (fr.device_id != d.device_id || fr.action_id != s_fireAct) break;
                    d.fire_success = fr.success;
                    Serial.printf("[master] FireResponse dev %lu success=%d fired=%u failed=%u\n",
                                  (unsigned long)d.device_id, fr.success,
                                  fr.slots_fired_count, fr.slots_failed_count);
                    advanceFire();
                    break;
                }
                case EvKind::LmlError:
                    s_opDevices[s_opDevIdx].fire_lml_error = true;
                    advanceFire();
                    break;
                case EvKind::FireTimeout:
                    s_opDevices[s_opDevIdx].fire_timeout = true;
                    advanceFire();
                    break;
                default: break;
            }
            break;

        case OpSub::ShowResult:
            if (ev.kind == EvKind::ButtonReleased) {
                char msg[32];
                buildResultMsg(msg, sizeof(msg));
                enterInit(msg);
            }
            break;
    }
}

// ── Buttons ──────────────────────────────────────────────────────────────────

struct Button {
    uint8_t       id;
    uint8_t       pin;
    QueueHandle_t queue;
    bool          stable;
    bool          lastRaw;
    uint32_t      lastEdgeMs;
};

static constexpr uint32_t BTN_DEBOUNCE_MS = 30;

static void buttonInit(Button &b, uint8_t id, uint8_t pin, QueueHandle_t q) {
    b.id        = id;  b.pin = pin;  b.queue = q;
    pinMode(pin, INPUT_PULLUP);
    delay(5);
    bool raw    = (digitalRead(pin) != 0);
    b.stable    = raw;  b.lastRaw = raw;  b.lastEdgeMs = millis();
}

static void buttonPoll(Button &b) {
    bool     raw = (digitalRead(b.pin) != 0);
    uint32_t now = millis();
    if (raw != b.lastRaw) { b.lastRaw = raw; b.lastEdgeMs = now; return; }
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

// ── App: state machine driver ────────────────────────────────────────────────

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
            case MState::Maintenance: handleMaintenance(ev); break;
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
    s_fireTimer = xTimerCreate("fire", pdMS_TO_TICKS(30000),
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
