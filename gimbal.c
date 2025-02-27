#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "ws2812.pio.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "hardware/gpio.h"
#include <math.h>
#include "hardware/pwm.h"

#define I2C_PORT i2c1
#define SDA_PIN 14
#define SCL_PIN 15
#define WS2812_PIN 7
#define WIDTH 128
#define HEIGHT 64
#define MATRIX_WIDTH 5
#define MATRIX_HEIGHT 5
#define MATRIX_LED_COUNT 25
#define MAX_DPS 250
#define DEBOUNCE_DELAY_US 350000
#define BUZZER_A 21
#define BUZZER_B 10
#define INCLINATION_THRESHOLD 100

// Variáveis globais
ssd1306_t ssd;
PIO pio = pio0;
uint sm;
uint32_t led_matrix[MATRIX_LED_COUNT];
volatile bool can_read_values = false;

void init_display() {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, 0x3C, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}

void setup_pwm(uint gpio, uint16_t wrap_value, float duty_cycle, float clkdiv) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(slice_num, clkdiv);
    pwm_set_wrap(slice_num, wrap_value);
    uint16_t level = (uint16_t)(wrap_value * duty_cycle);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), level);
    pwm_set_enabled(slice_num, true);
}

void disable_pwm(uint gpio) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_enabled(slice_num, false);
}

void gpio5_isr(uint gpio, uint32_t events) {
    gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_FALL, false);
    static uint64_t last_interrupt_time = 0;
    uint64_t current_time = time_us_64();
    if ((current_time - last_interrupt_time) > DEBOUNCE_DELAY_US) {
        can_read_values = true;
        last_interrupt_time = current_time;
    }
    gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_FALL, true);
}

void init_ws2812() {
    uint offset = pio_add_program(pio, &ws2812_program);
    sm = pio_claim_unused_sm(pio, true);
    ws2812_program_init(pio, sm, offset, WS2812_PIN);
}

int get_pixel_index(int row, int col) {
    if (row % 2 == 0) {
        return (MATRIX_HEIGHT - 1 - row) * MATRIX_WIDTH + (MATRIX_WIDTH - 1 - col);
    } else {
        return (MATRIX_HEIGHT - 1 - row) * MATRIX_WIDTH + col;
    }
}


void update_leds(int16_t x, int16_t y) {
    // Zera a matriz
    memset(led_matrix, 0, sizeof(led_matrix));

    // Iluminação baseada em X (vertical)
    // Calcula quantas linhas iluminar proporcionalmente à magnitude de X
    int x_rows = (abs(x) * MATRIX_HEIGHT) / MAX_DPS;
    if (x_rows > MATRIX_HEIGHT) x_rows = MATRIX_HEIGHT;
    // Define a cor: X positivo = vermelho, X negativo = verde
    uint32_t color_x = (x > 0) ? 0xFF0000 : 0x00FF00;
    for (int i = 0; i < x_rows; i++) {
        // Se X positivo, ilumina da base para o topo; se negativo, do topo para a base
        int row = (x > 0) ? (MATRIX_HEIGHT - 1 - i) : i;
        for (int col = 0; col < MATRIX_WIDTH; col++) {
            int index = get_pixel_index(row, col);
            if (index >= 0) {
                led_matrix[index] |= color_x;
            }
        }
    }

    // Iluminação baseada em Y (horizontal)
    // Calcula quantas colunas iluminar proporcionalmente à magnitude de Y
    int y_cols = (abs(y) * MATRIX_WIDTH) / MAX_DPS;
    if (y_cols > MATRIX_WIDTH) y_cols = MATRIX_WIDTH;
    // Define a cor: Y positivo = azul, Y negativo = amarelo
    uint32_t color_y = (y > 0) ? 0x0000FF : 0xFFFF00;
    for (int i = 0; i < y_cols; i++) {
        // Se Y positivo, ilumina da direita para a esquerda; se negativo, da esquerda para a direita
        int col = (y > 0) ? (MATRIX_WIDTH - 1 - i) : i;
        for (int row = 0; row < MATRIX_HEIGHT; row++) {
            int index = get_pixel_index(row, col);
            if (index >= 0) {
                led_matrix[index] |= color_y;
            }
        }
    }

    // Atualiza a matriz de LEDs via PIO
    for (int i = 0; i < MATRIX_LED_COUNT; i++) {
        pio_sm_put_blocking(pio, sm, led_matrix[i] << 8u);
    }
}

void display_gyro_data(int16_t x, int16_t y, int16_t z) {
    char buffer[32];
    ssd1306_fill(&ssd, false);
    snprintf(buffer, sizeof(buffer), "X: %d dps", x);
    ssd1306_draw_string(&ssd, buffer, 0, 0);
    snprintf(buffer, sizeof(buffer), "Y: %d dps", y);
    ssd1306_draw_string(&ssd, buffer, 0, 16);
    snprintf(buffer, sizeof(buffer), "Z: %d dps", z);
    ssd1306_draw_string(&ssd, buffer, 0, 32);
    ssd1306_send_data(&ssd);
}

void stabilize_values(int16_t *value) {
    if (*value > 0) (*value)--;
    else if (*value < 0) (*value)++;
}

void process_serial_input(char *input, int16_t *x, int16_t *y, int16_t *z) {
    sscanf(input, "X:%hd Y:%hd Z:%hd", x, y, z);
}

int main() {
    stdio_init_all();
    sleep_ms(100);
    init_display();
    init_ws2812();

    int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;
    char input_buffer[64];

    gpio_init(5);
    gpio_set_dir(5, GPIO_IN);
    gpio_pull_up(5);
    gpio_set_irq_enabled_with_callback(5, GPIO_IRQ_EDGE_FALL, true, &gpio5_isr);

    while (true) {
        if (can_read_values) {
            printf("\nDigite os valores no formato X:30 Y:20 Z:00 e pressione Enter:\n");
            int i = 0;
            char ch;
            while (i < sizeof(input_buffer) - 1) {
                ch = getchar();
                putchar(ch);
                if (ch == '\n' || ch == '\r') break;
                input_buffer[i++] = ch;
            }
            input_buffer[i] = '\0';

            process_serial_input(input_buffer, &gyro_x, &gyro_y, &gyro_z);

            while (gyro_x != 0 || gyro_y != 0) {
                stabilize_values(&gyro_x);
                stabilize_values(&gyro_y);
                display_gyro_data(gyro_x, gyro_y, gyro_z);
                update_leds(gyro_x, gyro_y);

                if (abs(gyro_x) > INCLINATION_THRESHOLD) {
                    setup_pwm(BUZZER_A, 1250, 0.5f, 100.0f);
                } else {
                    disable_pwm(BUZZER_A);
                }

                if (abs(gyro_y) > INCLINATION_THRESHOLD) {
                    setup_pwm(BUZZER_B, 1250, 0.5f, 100.0f);
                } else {
                    disable_pwm(BUZZER_B);
                }

                sleep_ms(100);
            }

            disable_pwm(BUZZER_A);
            disable_pwm(BUZZER_B);
            can_read_values = false;
        }
        sleep_ms(10);
    }
}

