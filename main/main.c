#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_memory_utils.h"
#include "driver/uart.h"
#include "string.h"
#include <stdlib.h>
#include <ctype.h>

// Конфигурация UART для RFID
#define RFID_UART_PORT UART_NUM_1
#define RFID_RX_PIN 21
#define RFID_TX_PIN 20
#define BUF_SIZE 1024

// Задача для чтения данных из UART (RFID)
void rfid_uart_read_task(void *pvParameters) {
    ESP_LOGI("RFID", "UART read task started");

    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE("RFID", "Failed to allocate memory for UART buffer");
        vTaskDelete(NULL);
    }

    while (1) {
        int len = uart_read_bytes(RFID_UART_PORT, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            ESP_LOGI("RFID", "Received %d bytes:", len);
            for (int i = 0; i < len; i++) {
                ESP_LOGI("RFID", "0x%02X ", data[i]);
            }

            if (len >= 14 && data[0] == 0x02 && data[13] == 0x03) {
                char card_id[11];
                for (int i = 0; i < 10; i++) {
                    card_id[i] = (char)data[i + 1];
                }
                card_id[10] = '\0';

                ESP_LOGI("RFID", "Raw card ID: %s", card_id);

                char hex_value[9];
                strncpy(hex_value, card_id + 2, 8);
                hex_value[8] = '\0';

                unsigned long dec_value = strtoul(hex_value, NULL, 16);

                char decrypted_id[11];
                snprintf(decrypted_id, sizeof(decrypted_id), "%010lu", dec_value);

                ESP_LOGI("RFID", "Hex value: %s", hex_value);
                ESP_LOGI("RFID", "Decrypted ID: %s", decrypted_id);

            } else {
                ESP_LOGE("RFID", "Invalid data format");
            }
        } else if (len == -1) {
            ESP_LOGE("RFID", "UART read error");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    free(data);
}

void app_main(void)
{
    // Инициализация NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Инициализация UART для RFID
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ret = uart_driver_install(RFID_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE("UART", "Failed to install UART driver: %s", esp_err_to_name(ret));
        return;
    }

    ret = uart_param_config(RFID_UART_PORT, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE("UART", "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return;
    }

    ret = uart_set_pin(RFID_UART_PORT, RFID_TX_PIN, RFID_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE("UART", "Failed to set UART pins: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI("UART", "UART initialized successfully");

    // Запускаем задачу для чтения данных из UART (RFID)
    xTaskCreate(rfid_uart_read_task, "rfid_uart_read_task", 2048, NULL, 10, NULL);
}
