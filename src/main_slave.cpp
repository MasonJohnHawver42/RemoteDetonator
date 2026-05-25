#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <LML.h>

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

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, OLED_RST, OLED_SCL, OLED_SDA);
SX1262       radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
LML::P2PAsync mac(radio, LORA_DIO1);

static SemaphoreHandle_t displayMutex;
static char dispLine1[32] = "waiting...";
static char dispLine2[32] = "";

static void setDisplay(const char *l1, const char *l2) {
    xSemaphoreTake(displayMutex, portMAX_DELAY);
    strncpy(dispLine1, l1, sizeof(dispLine1) - 1);
    strncpy(dispLine2, l2, sizeof(dispLine2) - 1);
    xSemaphoreGive(displayMutex);
}

static uint64_t computeFib(uint8_t n) {
    if (n == 0) return 0;
    if (n == 1) return 1;
    uint64_t a = 0, b = 1;
    for (uint8_t i = 2; i <= n; i++) {
        uint64_t c = a + b; a = b; b = c;
    }
    return b;
}

void displayTask(void *) {
    display.begin();
    display.setFont(u8g2_font_ncenB08_tr);
    char l1[32], l2[32];
    while (true) {
        xSemaphoreTake(displayMutex, portMAX_DELAY);
        strncpy(l1, dispLine1, sizeof(l1));
        strncpy(l2, dispLine2, sizeof(l2));
        xSemaphoreGive(displayMutex);
        display.clearBuffer();
        display.drawStr(0, 12, "[ SLAVE ]");
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

void appTask(void *) {
    Serial.println("[slave] appTask start");

    while (true) {
        LML::Event ev{};
        xQueueReceive(mac.rx_queue, &ev, portMAX_DELAY);

        if (ev.tag == LML::Event::Tag::Error) {
            Serial.printf("[slave] event error err=%d — ignoring\n", (int)ev.err);
            setDisplay("tx error", "");
            continue;
        }

        const LMLMessage &msg = ev.msg;
        Serial.printf("[slave] received %u bytes\n", msg.len);

        if (msg.len != 1) {
            Serial.printf("[slave] unexpected len=%u — ignoring\n", msg.len);
            continue;
        }

        uint8_t n = msg.data[0];
        Serial.printf("[slave] n=%u computing fib...\n", n);

        uint64_t fib = computeFib(n);
        Serial.printf("[slave] fib(%u)=%llu\n", n, fib);

        char l1[32], l2[32];
        snprintf(l1, sizeof(l1), "n=%u", n);
        snprintf(l2, sizeof(l2), "f=%lu", (unsigned long)(fib & 0xFFFFFFFF));
        setDisplay(l1, l2);

        // Retry queue_tx until accepted (poll() may still be finishing
        // a previous exchange when appTask runs).
        LMLError err;
        do {
            err = mac.queue_tx((const uint8_t *)&fib, sizeof(fib));
            if (err != LMLError::OK) {
                Serial.printf("[slave] queue_tx busy err=%d — retrying\n", (int)err);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        } while (err != LMLError::OK);
        Serial.println("[slave] fib queued for TX");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("[slave] boot");

    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, LOW);
    delay(100);

    displayMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(protocolTask, "lml_s", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(appTask,      "app_s", 4096, NULL, 2, NULL, 1);
    xTaskCreate(displayTask, "display",   4096, NULL, 1, NULL);

    Serial.println("[slave] tasks created");
}

void loop() { vTaskDelay(portMAX_DELAY); }
