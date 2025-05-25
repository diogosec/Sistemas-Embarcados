#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"

#define LED_0 GPIO_NUM_36
#define LED_1 GPIO_NUM_37
#define LED_2 GPIO_NUM_38
#define LED_3 GPIO_NUM_39

#define BTN_A GPIO_NUM_48
#define BTN_B GPIO_NUM_47

#define DEBOUNCE_TIME_MS 500

static int contador = 0;
static int incremento = 1;

static int64_t last_press_a = 0;
static int64_t last_press_b = 0;


static QueueHandle_t gpio_evt_queue = NULL;

typedef enum {
    BTN_A_PRESSED,
    BTN_B_PRESSED
} button_event_t;

void atualiza_leds(int valor) {
    gpio_set_level(LED_0, (valor & 0x01));
    gpio_set_level(LED_1, (valor & 0x02) >> 1);
    gpio_set_level(LED_2, (valor & 0x04) >> 2);
    gpio_set_level(LED_3, (valor & 0x08) >> 3);
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    int64_t now = esp_timer_get_time() / 1000;

    if (gpio_num == BTN_A && (now - last_press_a) > DEBOUNCE_TIME_MS) {
        last_press_a = now;
        button_event_t evt = BTN_A_PRESSED;
        xQueueSendFromISR(gpio_evt_queue, &evt, NULL);
    } else if (gpio_num == BTN_B && (now - last_press_b) > DEBOUNCE_TIME_MS) {
        last_press_b = now;
        button_event_t evt = BTN_B_PRESSED;
        xQueueSendFromISR(gpio_evt_queue, &evt, NULL);
    }
}

void button_task(void* arg) {
    button_event_t evt;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &evt, portMAX_DELAY)) {
            if (evt == BTN_A_PRESSED) {
                contador = (contador + incremento) & 0x0F;
                atualiza_leds(contador);
            } else if (evt == BTN_B_PRESSED) {
                incremento = (incremento == 1) ? 2 : 1;
            }
        }
    }
}

void app_main(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_0) | (1ULL << LED_1) | (1ULL << LED_2) | (1ULL << LED_3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << BTN_A) | (1ULL << BTN_B);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(button_event_t));

    //Serviço de interrupção GPIO
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_A, gpio_isr_handler, (void*) BTN_A);
    gpio_isr_handler_add(BTN_B, gpio_isr_handler, (void*) BTN_B);

    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
}
