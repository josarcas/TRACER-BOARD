#ifndef INC_SCREEN_H
#define INC_SCREEN_H
/*INCLUDES********************************************************************************************/
#include "hardware/i2c.h"

#include "stdint.h"

/*DEFINES*********************************************************************************************/
#ifndef I2C_BUS
#define I2C_BUS
#define I2C_PORT                    i2c0
#define I2C_BAUD                    1000000
#define SDA_PIN                     16
#define SCL_PIN                     17
#endif

#define OLED_DIR                    60
#define OLED_DISPLAY_ON             0xAF
#define OLED_DISPLAY_OFF            0xAE
#define OLED_NORMAL_COLOR           0xA6           
#define OLED_INVERT_COLOR           0xA7
#define OLED_STOP_SCROLL            0x2E
#define OLED_START_SCROLL           0x2F


#define OLED_ROW_SIZE               128
#define OLED_PAGE_SIZE              8
#define OLED_HEIGHT_SIZE            64
#define OLED_WIDTH_SIZE             128

/*TYPEDEFS********************************************************************************************/
typedef enum{
    H_RIGHT         = 0x26,
    H_LEFT          = 0x27,
    V_RIGHT         = 0x29,
    V_LEFT          = 0x2A
}scroll_dir_t;

typedef enum{
    PAGE_0          = 0x00,
    PAGE_1          = 0x01,
    PAGE_2          = 0x02,
    PAGE_3          = 0x03,
    PAGE_4          = 0x04,
    PAGE_5          = 0x05,
    PAGE_6          = 0x06,
    PAGE_7          = 0x07
}oled_page_t;

typedef enum{
    FRAMES_5      = 0x00,
	FRAMES_64     = 0x01,
	FRAMES_128    = 0x02,
	FRAMES_256    = 0x03,
	FRAMES_3      = 0x04,
	FRAMES_4      = 0x05,
	FRAMES_25     = 0x06,
	FRAMES_2      = 0x07
}frame_rate_t;

/*PROTOTYPES*******************************************************************************************/
void oled_init();
void oled_refresh();
void oled_set_contrast(uint8_t contrast);
void oled_set_display_on(bool on);
void oled_set_horizontal_scroll(scroll_dir_t dir, oled_page_t start, oled_page_t end,
                                    frame_rate_t frame_rate);
void oled_set_start_scroll(bool start);
void oled_set_pixel(uint8_t x, uint8_t y);
void oled_reset_pixel(uint8_t x, uint8_t y);
void oled_clear();
void oled_draw_string(char *str, uint8_t x, uint8_t y);
void oled_draw_bitmap(const uint8_t bitmap, uint8_t x, uint8_t y, uint8_t width, uint16_t size);


#endif