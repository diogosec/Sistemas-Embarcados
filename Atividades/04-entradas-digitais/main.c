#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#define LED_0 GPIO_NUM_36
#define LED_1 GPIO_NUM_37
#define LED_2 GPIO_NUM_38
#define LED_3 GPIO_NUM_39

#define BTN_A GPIO_NUM_48
#define BTN_B GPIO_NUM_47

#define DEBOUNCE_TIME_MS 50 

int contador = 0;
int incremento = 1;

/*void print_binario_decimal(int valor) {
    printf("Decimal: %2d\n", valor);
}*/

void atualiza_leds(int valor) {
    gpio_set_level(LED_0, (valor & 0x01));
    gpio_set_level(LED_1, (valor & 0x02) >> 1);
    gpio_set_level(LED_2, (valor & 0x04) >> 2);
    gpio_set_level(LED_3, (valor & 0x08) >> 3);
    //print_binario_decimal(valor);
}

void app_main() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_0) | (1ULL << LED_1) | (1ULL << LED_2) | (1ULL << LED_3),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << BTN_A) | (1ULL << BTN_B);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    int last_state_a = 1, last_state_b = 1;
    int stable_state_a = 1, stable_state_b = 1;

    int64_t last_debounce_a = 0;
    int64_t last_debounce_b = 0;

    while (1) {
        int curr_state_a = gpio_get_level(BTN_A);
        int curr_state_b = gpio_get_level(BTN_B);

        int64_t now = esp_timer_get_time() / 1000; //ms

        //Debounce botao a
        if (curr_state_a != last_state_a) {
            last_debounce_a = now;
            last_state_a = curr_state_a;
        }
        if ((now - last_debounce_a) > DEBOUNCE_TIME_MS && curr_state_a != stable_state_a) {
            stable_state_a = curr_state_a;
            if (stable_state_a == 1) { //borda de subida
                contador = (contador + incremento) & 0x0F;
                atualiza_leds(contador);
            }
        }

        //Debounce botao b
        if (curr_state_b != last_state_b) {
            last_debounce_b = now;
            last_state_b = curr_state_b;
        }
        if ((now - last_debounce_b) > DEBOUNCE_TIME_MS && curr_state_b != stable_state_b) {
            stable_state_b = curr_state_b;
            if (stable_state_b == 1) {
                incremento = (incremento == 1) ? 2 : 1;
                //printf("Incremento alterado para +%d\n", incremento);
            }
        }
    }
}