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

void appTask(void *) {
    Serial.println("[master] appTask start — waiting 2s");
    vTaskDelay(pdMS_TO_TICKS(2000));

    uint8_t n = 0;
    char line1[32], line2[32];

    // Queue first request
    snprintf(line1, sizeof(line1), "sending n=%u", n);
    setDisplay(line1, "...");
    mac.queue_tx(&n, 1);
    Serial.printf("[master] queued n=%u\n", n);

    while (true) {
        LML::Event ev{};
        xQueueReceive(mac.rx_queue, &ev, portMAX_DELAY);

        if (ev.tag == LML::Event::Tag::Error) {
            Serial.printf("[master] send error err=%d — retrying n=%u\n",
                          (int)ev.err, n);
            snprintf(line1, sizeof(line1), "err n=%u", n);
            setDisplay(line1, "retrying...");
            vTaskDelay(pdMS_TO_TICKS(300));
            mac.queue_tx(&n, 1);
            continue;
        }

        // Received a message — should be fib(n) as uint64_t
        const LMLMessage &msg = ev.msg;
        if (msg.len != sizeof(uint64_t)) {
            Serial.printf("[master] bad response len=%u — retrying n=%u\n", msg.len, n);
            snprintf(line1, sizeof(line1), "bad len n=%u", n);
            setDisplay(line1, "retrying...");
            vTaskDelay(pdMS_TO_TICKS(300));
            mac.queue_tx(&n, 1);
            continue;
        }

        uint64_t fib;
        memcpy(&fib, msg.data, sizeof(fib));
        Serial.printf("[master] fib(%u)=%llu  rssi=%.1fdBm\n", n, fib, msg.rssi);

        snprintf(line1, sizeof(line1), "n=%u", n);
        snprintf(line2, sizeof(line2), "f=%lu", (unsigned long)(fib & 0xFFFFFFFF));
        setDisplay(line1, line2);

        // Advance to next n and send
        n = (n + 1) & 0xFF;
        vTaskDelay(pdMS_TO_TICKS(100));
        mac.queue_tx(&n, 1);
        Serial.printf("[master] queued n=%u\n", n);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("[master] boot");

    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, LOW);
    delay(100);

    displayMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(protocolTask, "lml_m", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(appTask,      "app_m", 4096, NULL, 2, NULL, 1);
    xTaskCreate(displayTask, "display",   4096, NULL, 1, NULL);

    Serial.println("[master] tasks created");
}

void loop() { vTaskDelay(portMAX_DELAY); }
