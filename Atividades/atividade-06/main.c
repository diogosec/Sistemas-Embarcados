#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "int_i2c.h"

#define LOG_TAG "DISPLAY_CONTADOR"

// Pinos

#define PIN_LED_A 39
#define PIN_LED_B 38
#define PIN_LED_C 37
#define PIN_LED_D 36
#define PIN_BTN_UP 4
#define PIN_BTN_DOWN 7
#define PIN_LED_PWM 35
#define PIN_SDA 8
#define PIN_SCL 9

#define I2C_CHANNEL I2C_NUM_0
#define LCD_I2C_ADDRESS 0x27


typedef enum {
    EVENTO_INCREMENTO,
    EVENTO_DECREMENTO
} tipo_evento_t;

static uint8_t valor = 0;
static QueueHandle_t fila_eventos = NULL;

lcd_i2c_handle_t visor = {
    .address = LCD_I2C_ADDRESS,
    .num = I2C_CHANNEL,
    .backlight = 1,
    .size = DISPLAY_16X02
};

// Interruptores
static void IRAM_ATTR isr_botao_up(void* arg) {
    tipo_evento_t e = EVENTO_INCREMENTO;
    xQueueSendFromISR(fila_eventos, &e, NULL);
}

static void IRAM_ATTR isr_botao_down(void* arg) {
    tipo_evento_t e = EVENTO_DECREMENTO;
    xQueueSendFromISR(fila_eventos, &e, NULL);
}

// LED binário
void exibir_em_leds(uint8_t val) {
    gpio_set_level(PIN_LED_A, val & 0x01);
    gpio_set_level(PIN_LED_B, (val >> 1) & 0x01);
    gpio_set_level(PIN_LED_C, (val >> 2) & 0x01);
    gpio_set_level(PIN_LED_D, (val >> 3) & 0x01);
}

// PWM proporcional
void atualizar_brilho_led(uint8_t nivel) {
    uint32_t duty_cycle = (nivel * 8191) / 15;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_cycle);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// Visor I2C
void mostrar_no_lcd(uint8_t numero) {
    lcd_i2c_cursor_set(&visor, 0, 0);
    lcd_i2c_print(&visor, "HEX: 0x%X", numero);

    lcd_i2c_cursor_set(&visor, 0, 1);
    lcd_i2c_print(&visor, "DEC:     ");
    lcd_i2c_cursor_set(&visor, 0, 1);
    lcd_i2c_print(&visor, "DEC: %d", numero);
}

void inicializar_botoes() {
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << PIN_BTN_UP) | (1ULL << PIN_BTN_DOWN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_BTN_UP, isr_botao_up, NULL);
    gpio_isr_handler_add(PIN_BTN_DOWN, isr_botao_down, NULL);
}

void inicializar_pwm() {
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_config = {
        .gpio_num = PIN_LED_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&pwm_config);
}

void inicializar_lcd() {
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_CHANNEL, &i2c_cfg);
    i2c_driver_install(I2C_CHANNEL, I2C_MODE_MASTER, 0, 0, 0);
    lcd_i2c_init(&visor);
}

void app_main(void) {
    gpio_set_direction(PIN_LED_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_C, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED_D, GPIO_MODE_OUTPUT);

    inicializar_botoes();
    inicializar_pwm();
    inicializar_lcd();

    fila_eventos = xQueueCreate(10, sizeof(tipo_evento_t));

    exibir_em_leds(valor);
    atualizar_brilho_led(valor);
    mostrar_no_lcd(valor);

    while (1) {
        tipo_evento_t evento_recebido;
        if (xQueueReceive(fila_eventos, &evento_recebido, pdMS_TO_TICKS(20))) {
            static int64_t tempo_anterior = 0;
            int64_t agora = esp_timer_get_time();

            if (agora - tempo_anterior > 400000) {
                tempo_anterior = agora;

                if (evento_recebido == EVENTO_INCREMENTO) {
                    valor = (valor + 1) & 0x0F;
                } else if (evento_recebido == EVENTO_DECREMENTO) {
                    valor = (valor - 1) & 0x0F;
                }

                exibir_em_leds(valor);
                atualizar_brilho_led(valor);
                mostrar_no_lcd(valor);
                vTaskDelay(pdMS_TO_TICKS(50));

                ESP_LOGI(LOG_TAG, "Contador: 0x%02X (%d)", valor, valor);
            }
        }
    }
}
