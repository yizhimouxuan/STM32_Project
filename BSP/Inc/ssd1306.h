#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT   64

typedef enum {
    SSD1306_COLOR_BLACK = 0x00,
    SSD1306_COLOR_WHITE = 0x01
} ssd1306_color_t;

bool ssd1306_init(I2C_HandleTypeDef *i2c);
void ssd1306_fill(ssd1306_color_t color);
void ssd1306_update(void);
void ssd1306_set_cursor(uint8_t x, uint8_t y);
bool ssd1306_write_string(const char *str, ssd1306_color_t color);
void ssd1306_display_metrics(float distance_m,
                             float right_dist_m,
                             float right_front_dist_m,
                             float left_dist_m,
                             float roll_deg,
                             float pitch_deg,
                             float yaw_deg);

#endif /* __SSD1306_H__ */
