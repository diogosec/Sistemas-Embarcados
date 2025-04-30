#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED1_GPIO 2
#define LED2_GPIO 3

void pisca_led1(void *pvParameter) {
    while (1) {
        gpio_set_level(LED1_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED1_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void pisca_led2(void *pvParameter) {
    while (1) {
        gpio_set_level(LED2_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED2_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main() {
    gpio_set_direction(LED1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2_GPIO, GPIO_MODE_OUTPUT);

    xTaskCreate(pisca_led1, "LED1", 1024, NULL, 1, NULL);
    xTaskCreate(pisca_led2, "LED2", 1024, NULL, 1, NULL);
}
