#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <Wire.h>

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
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

static SemaphoreHandle_t displayMutex;
static char dispLine1[32] = "";
static char dispLine2[32] = "";

static void setStatus(const char *l1, const char *l2) {
    xSemaphoreTake(displayMutex, portMAX_DELAY);
    strncpy(dispLine1, l1, sizeof(dispLine1) - 1);
    strncpy(dispLine2, l2, sizeof(dispLine2) - 1);
    xSemaphoreGive(displayMutex);
}

void displayTask(void *pvParameters) {
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

void radioTask(void *pvParameters) {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

    int state = radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8, 1.8);
    if (state != RADIOLIB_ERR_NONE) {
        char err[32];
        snprintf(err, sizeof(err), "init err: %d", state);
        setStatus(err, "");
        vTaskDelete(NULL);
    }
    radio.setDio2AsRfSwitch(true);

    uint32_t counter = 0;
    char packet[32];
    char result[32];

    while (true) {
        snprintf(packet, sizeof(packet), "PING %lu", counter++);
        setStatus(packet, "sending...");

        state = radio.transmit(packet);
        if (state == RADIOLIB_ERR_NONE) {
            setStatus(packet, "sent OK");
        } else {
            snprintf(result, sizeof(result), "TX err: %d", state);
            setStatus(packet, result);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, LOW);
    delay(100);

    displayMutex = xSemaphoreCreateMutex();

    xTaskCreate(displayTask, "display", 4096, NULL, 1, NULL);
    xTaskCreate(radioTask,   "radio",   8192, NULL, 2, NULL);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
