#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_adc/adc_oneshot.h"
#include "int_i2c.h"

#define TAG "TEMP_SD"

// GPIOs
#define PIN_LED1 39
#define PIN_LED2 37
#define PIN_LED3 38
#define PIN_LED4 36
#define PIN_BTN_INC 4
#define PIN_BTN_DEC 7
#define PIN_BUZZER 35
#define PIN_SDA 8
#define PIN_SCL 9
#define NTC_GPIO 10
#define NTC_CHANNEL ADC_CHANNEL_9

// SDCard SPI
#define PIN_NUM_MISO 11
#define PIN_NUM_MOSI 12
#define PIN_NUM_CLK  13
#define PIN_NUM_CS   14

#define TEMP_ALARME_DEFAULT 25
#define DEBOUNCE_US 200000

typedef enum {
    EVENTO_INC,
    EVENTO_DEC
} tipo_evento_t;

typedef enum {
    ESTADO_MONITORANDO,
    ESTADO_ALARME_ATIVO
} estado_t;

static QueueHandle_t fila_eventos = NULL;
static int temp_alarme = TEMP_ALARME_DEFAULT;
static int64_t ultimo_evento_us = 0;
static int64_t ultimo_pisca_us = 0;
static bool estado_pisca = false;
static bool buzzer_ativo = false;
static estado_t estado_atual = ESTADO_MONITORANDO;

lcd_i2c_handle_t lcd = {
    .address = 0x27,
    .num = I2C_NUM_0,
    .backlight = 1,
    .size = DISPLAY_16X02
};

adc_oneshot_unit_handle_t adc_handle;

// ISR com debounce
static void IRAM_ATTR isr_btn_inc(void* arg) {
    int64_t agora = esp_timer_get_time();
    if (agora - ultimo_evento_us > DEBOUNCE_US) {
        tipo_evento_t evt = EVENTO_INC;
        xQueueSendFromISR(fila_eventos, &evt, NULL);
        ultimo_evento_us = agora;
    }
}

static void IRAM_ATTR isr_btn_dec(void* arg) {
    int64_t agora = esp_timer_get_time();
    if (agora - ultimo_evento_us > DEBOUNCE_US) {
        tipo_evento_t evt = EVENTO_DEC;
        xQueueSendFromISR(fila_eventos, &evt, NULL);
        ultimo_evento_us = agora;
    }
}

int ler_temperatura_ntc() {
    const float BETA = 3950.0;
    int raw = 0;
    adc_oneshot_read(adc_handle, NTC_CHANNEL, &raw);
    float celsius = 1.0 / (log(1.0 / (4095.0 / raw - 1.0)) / BETA + 1.0 / 298.15) - 273.15;
    return (int)celsius;
}

void atualizar_lcd(int temp, int limite) {
    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_print(&lcd, "NTC:%2dC Al:%2dC ", temp, limite);
}

void atualizar_leds(bool piscar, int diff) {
    if (piscar) {
        int64_t agora = esp_timer_get_time();
        if (agora - ultimo_pisca_us > 500000) {
            ultimo_pisca_us = agora;
            estado_pisca = !estado_pisca;
        }
        gpio_set_level(PIN_LED1, estado_pisca);
        gpio_set_level(PIN_LED2, estado_pisca);
        gpio_set_level(PIN_LED3, estado_pisca);
        gpio_set_level(PIN_LED4, estado_pisca);
    } else {
        gpio_set_level(PIN_LED1, diff <= 20);
        gpio_set_level(PIN_LED2, diff <= 15);
        gpio_set_level(PIN_LED3, diff <= 10);
        gpio_set_level(PIN_LED4, diff <= 2);
    }
}

void controlar_buzzer(bool ligar) {
    if (ligar && !buzzer_ativo) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        buzzer_ativo = true;
    } else if (!ligar && buzzer_ativo) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        buzzer_ativo = false;
    }
}

void salvar_sdcard(FILE* f, int temperatura) {
    if (f) {
        fprintf(f, "Temperatura: %d\n", temperatura);
        fflush(f);
    }
}

void inicializar_lcd() {
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &cfg);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    lcd_i2c_init(&lcd);
}

void inicializar_leds() {
    gpio_set_direction(PIN_LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LED4, GPIO_MODE_OUTPUT);
}

void inicializar_botoes() {
    gpio_config_t conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = (1ULL << PIN_BTN_INC) | (1ULL << PIN_BTN_DEC)
    };
    gpio_config(&conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_BTN_INC, isr_btn_inc, NULL);
    gpio_isr_handler_add(PIN_BTN_DEC, isr_btn_dec, NULL);
}

void inicializar_buzzer_pwm() {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 4000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = PIN_BUZZER,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel);
}

FILE* inicializar_sdcard() {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 3
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.flags = SDMMC_HOST_FLAG_SPI;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    sdmmc_card_t* card;
    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) return NULL;

    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) return NULL;

    FILE* f = fopen("/sdcard/temperatura.txt", "a");
    return f;
}

void inicializar_adc() {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1
    };
    adc_oneshot_new_unit(&init_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t cfg = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12
    };
    adc_oneshot_config_channel(adc_handle, NTC_CHANNEL, &cfg);
}

void app_main(void) {
    inicializar_leds();
    inicializar_lcd();
    inicializar_botoes();
    inicializar_buzzer_pwm();
    inicializar_adc();

    fila_eventos = xQueueCreate(10, sizeof(tipo_evento_t));
    FILE* arquivo_sd = inicializar_sdcard();

    int temperatura = 0;
    int temp_anterior = -999;
    int alarme_anterior = -999;

    while (1) {
        tipo_evento_t evt;
        if (xQueueReceive(fila_eventos, &evt, 0)) {
            if (evt == EVENTO_INC) temp_alarme += 5;
            if (evt == EVENTO_DEC) temp_alarme -= 5;
        }

        temperatura = ler_temperatura_ntc();

        switch (estado_atual) {
            case ESTADO_MONITORANDO:
                controlar_buzzer(false);
                atualizar_leds(false, temp_alarme - temperatura);
                if (temperatura >= temp_alarme) {
                    estado_atual = ESTADO_ALARME_ATIVO;
                }
                break;

            case ESTADO_ALARME_ATIVO:
                controlar_buzzer(true);
                atualizar_leds(true, 0);
                if (temperatura < temp_alarme) {
                    estado_atual = ESTADO_MONITORANDO;
                }
                break;
        }

        if (temperatura != temp_anterior || temp_alarme != alarme_anterior) {
            atualizar_lcd(temperatura, temp_alarme);
            salvar_sdcard(arquivo_sd, temperatura);
            temp_anterior = temperatura;
            alarme_anterior = temp_alarme;
        }

        ESP_LOGI(TAG, "NTC: %d | Alarme: %d | Estado: %s", temperatura, temp_alarme,
                 estado_atual == ESTADO_MONITORANDO ? "MONITORANDO" : "ALARME");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
