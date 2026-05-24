#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <MSRC.h>

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
SX1262    radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
MSRCSlave msrc(radio);

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
        uint64_t c = a + b;
        a = b;
        b = c;
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
    Serial.println("[slave] SPI init OK");

    int state = radio.begin(915.0, 125.0, 9, 7,
                            RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8, 1.8);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[slave] radio.begin FAILED state=%d\n", state);
        setDisplay("radio fail", "");
        vTaskDelete(NULL);
    }
    Serial.println("[slave] radio OK");

    radio.setDio2AsRfSwitch(true);
    msrc.init();
    msrc.setCTS(true);
    Serial.println("[slave] msrc init OK, CTS=true — entering poll loop");

    while (true) {
        msrc.poll(100);
    }
}

void appTask(void *) {
    Serial.println("[slave] appTask start");

    while (true) {
        MSRCMessage msg;
        if (msrc.read(msg) != MSRCError::OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        Serial.printf("[slave] received %u bytes  rssi=%.1fdBm\n", msg.len, msg.rssi);

        if (msg.len != 1) {
            Serial.printf("[slave] unexpected len=%u — ignoring\n", msg.len);
            continue;
        }

        uint8_t n = msg.data[0];
        Serial.printf("[slave] got n=%u, computing fib...\n", n);

        msrc.setCTS(false);
        Serial.println("[slave] CTS=false (busy)");

        uint64_t fib = computeFib(n);
        Serial.printf("[slave] fib(%u) = %llu\n", n, fib);

        char line1[32], line2[32];
        snprintf(line1, sizeof(line1), "n=%u", n);
        snprintf(line2, sizeof(line2), "f=%lu", (unsigned long)(fib & 0xFFFFFFFF));
        setDisplay(line1, line2);

        Serial.println("[slave] sending fib result...");
        MSRCError err = msrc.send((const uint8_t *)&fib, sizeof(fib));
        if (err != MSRCError::OK) {
            Serial.printf("[slave] send FAILED err=%d\n", (int)err);
        } else {
            Serial.println("[slave] send OK");
        }

        msrc.setCTS(true);
        Serial.println("[slave] CTS=true (ready)");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("[slave] boot");

    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, LOW);
    delay(100);

    displayMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(protocolTask, "msrc_s", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(appTask,      "app_s",  4096, NULL, 2, NULL, 1);
    xTaskCreate(displayTask, "display", 4096, NULL, 1, NULL);

    Serial.println("[slave] tasks created");
}

void loop() { vTaskDelay(portMAX_DELAY); }
