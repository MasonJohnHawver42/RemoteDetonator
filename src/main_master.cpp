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
SX1262     radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
MSRCMaster msrc(radio, LORA_DIO1);

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
    while (true) {
        xSemaphoreTake(displayMutex, portMAX_DELAY);
        strncpy(l1, dispLine1, sizeof(l1));
        strncpy(l2, dispLine2, sizeof(l2));
        xSemaphoreGive(displayMutex);
        display.clearBuffer();
        display.drawStr(0, 12, "[ MASTER ]");
        display.drawStr(0, 32, l1);
        display.drawStr(0, 48, l2);
        display.sendBuffer();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void protocolTask(void *) {
    Serial.println("[master] protocolTask start");
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    Serial.println("[master] SPI init OK");

    int state = radio.begin(915.0, 125.0, 9, 7,
                            RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8, 1.8);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[master] radio.begin FAILED state=%d\n", state);
        setDisplay("radio fail", "");
        vTaskDelete(NULL);
    }
    Serial.println("[master] radio OK");

    radio.setDio2AsRfSwitch(true);
    msrc.init();
    Serial.println("[master] msrc init OK — entering poll loop");

    while (true) {
        msrc.poll(300);
    }
}

void appTask(void *) {
    Serial.println("[master] appTask start — waiting 2s for radio");
    vTaskDelay(pdMS_TO_TICKS(2000));
    Serial.println("[master] appTask — entering fib loop");

    uint8_t n = 0;
    char line1[32], line2[32];

    while (true) {
        Serial.printf("[master] --- loop n=%u ---\n", n);

        msrc.setRTS(true);
        Serial.println("[master] RTS=true, waiting for CTS...");

        if (!msrc.waitCTS(3000)) {
            Serial.println("[master] CTS timeout — slave not responding");
            setDisplay("CTS timeout", "");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        Serial.println("[master] CTS granted");

        MSRCError err = msrc.send(&n, 1);
        if (err != MSRCError::OK) {
            Serial.printf("[master] send FAILED err=%d\n", (int)err);
            snprintf(line1, sizeof(line1), "send err %d", (int)err);
            setDisplay(line1, "");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        Serial.printf("[master] sent n=%u OK\n", n);
        snprintf(line1, sizeof(line1), "sent n=%u", n);
        setDisplay(line1, "waiting fib...");

        Serial.printf("[master] waiting for fib(%u) response...\n", n);
        MSRCMessage msg;
        uint32_t deadline = millis() + 5000;
        bool got = false;
        while (millis() < deadline) {
            if (msrc.read(msg) == MSRCError::OK) {
                got = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (!got) {
            Serial.printf("[master] TIMEOUT — no fib(%u) response\n", n);
            setDisplay(line1, "rx timeout");
            vTaskDelay(pdMS_TO_TICKS(300));
            continue; // retry same n — slave will drain its TX on next HB
        } else if (msg.len != sizeof(uint64_t)) {
            Serial.printf("[master] bad response len=%u expected 8\n", msg.len);
            setDisplay(line1, "bad len");
            vTaskDelay(pdMS_TO_TICKS(300));
            continue; // retry same n
        } else {
            uint64_t fib;
            memcpy(&fib, msg.data, sizeof(fib));
            Serial.printf("[master] fib(%u) = %llu  rssi=%.1fdBm\n", n, fib, msg.rssi);
            snprintf(line2, sizeof(line2), "f=%lu", (unsigned long)(fib & 0xFFFFFFFF));
            setDisplay(line1, line2);
        }

        n = (n + 1) & 0xFF;
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("[master] boot");

    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, LOW);
    delay(100);

    displayMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(protocolTask, "msrc_m", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(appTask,      "app_m",  4096, NULL, 2, NULL, 1);
    xTaskCreate(displayTask, "display", 4096, NULL, 1, NULL);

    Serial.println("[master] tasks created");
}

void loop() { vTaskDelay(portMAX_DELAY); }
