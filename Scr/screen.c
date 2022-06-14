/*INCLUDES********************************************************************************************/
#include "stdarg"

#include "FreeRTOS.h"

#include "screen.h"
#include "fonts.h"

/*GLOBAL VARIABLES************************************************************************************/
static uint8_t matrix[OLED_PAGE_SIZE][OLED_ROW_SIZE];

/*PROTOTYPES******************************************************************************************/
static void oled_write_command(uint8_t cmd);
static void oled_write_l_command(uint8_t *cmd, uint16_t len);
static void oled_write_data(uint8_t data);
static void oled_write_page(uint8_t page, uint8_t *data);
static uint8_t reverse(uint8_t b);

/*FUNCTIONS*******************************************************************************************/

/*  \brief  Write command to oled screen.
 *  
 *  \param  cmd     Command for write to screen.
 * 
 */
static void oled_write_command(uint8_t cmd)
{
    uint8_t buffer[2] = {0x00, cmd};
    i2c_write_blocking(I2C_PORT, OLED_DIR, buffer, 2, false);
}

/*  \brief  Write multi-command to oled screen.
 *  
 *  \param  cmd     Pointer of command for write to screen.
 *  \param  len     Size of command list.
 * 
 */
static void oled_write_l_command(uint8_t *cmd, uint16_t len)
{
    uint8_t *buffer;

    buffer = pvPortMalloc(len+1);
    buffer[0] = 0x00;

    for(uint16_t i=0; i<len; i++)
        buffer[i+1] = cmd[i];

    i2c_write_blocking(I2C_PORT, I2C_DIR_OLED, buffer, len+1, false);

    vPortFree(buffer);    
}

/*  \brief  Write data to oled screen.
 *  
 *  \param  data        Data for write to screen.
 * 
 */
static void oled_write_data(uint8_t data)
{
    uint8_t buffer[2] = {0x00, data};
    i2c_write_blocking(I2C_PORT, OLED_DIR, buffer, 2, false);    
}

/*  \brief  Write data to page on screen (see datasheet).
 *  
 *  \param  page        Number of page to write.
 *  \param  data        Pointer to data to write.
 * 
 */
static void oled_write_page(uint8_t page, uint8_t *data)
{
    uint8_t *buffer;

    buffer = pvPortMalloc(OLED_ROW_SIZE+1)

    buffer[0] = 0x40;
	for(uint8_t i=0; i<OLED_ROW_SIZE; i++)
		buffer[i+1] = data[i];

	write_command_oled(0xb0 + page);
	write_command_oled(0x00);
	write_command_oled(0x10);

    i2c_write_blocking(I2C_PORT, I2C_DIR_OLED, buffer, OLED_ROW_SIZE+1, false);

    vPortFree(buffer);
}

/*  \brief  Invert bits of byte.
 *  
 *  \param  b       Byte for invert.
 * 
 *  \return Byte invert.
 * 
 */
static uint8_t reverse(uint8_t b)
{
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

/*  \brief  Initialize screen.
 * 
 */
void oled_init()
{
    oled_write_command(0xa8);
	oled_write_command(0x3f);
	oled_write_command(0xd3);
	oled_write_command(0x00);
	oled_write_command(0x40);
	oled_write_command(0xa1);
	oled_write_command(0xc8);
	oled_write_command(0xad);
	oled_write_command(0x02);
	oled_write_command(0x81);
	oled_write_command(0x7f);
	oled_write_command(0xa4);
	oled_write_command(0xa6);
	oled_write_command(0xd5);
	oled_write_command(0x80);
	oled_write_command(0x8d);
	oled_write_command(0x14);
	oled_write_command(0xaf);
}

/*  \brief  Refresh screen.
 * 
 */
void oled_refresh()
{
    for(uint16_t i=0; i<NUM_PAGES; i++)
        write_page_oled(i, matrix[i]);
}

/*  \brief  Set screen constrast.
 *
 *  \param  contrast     Value of contrast.
 *  
 */
void oled_set_contrast(uint8_t contrast)
{
    uint8_t buffer[2] = {0x81, contrast};
    oled_write_l_command(buffer, 2);
}

/*  \brief  Set on/off screen.
 *
 *  \param  on      1 for display on or 0 for display off.
 *  
 */
void oled_set_display_on(bool on)
{
    if(on)
        oled_write_command(OLED_DISPLAY_ON);
    else
        oled_write_command(OLED_DISPLAY_OFF);

}

/*  \brief  Set screen constrast.
 *
 *  \param  dir             Directon of scroll.
 *  \param  start           Page start.
 *  \param  end             Page end.
 *  \param  frame_rate      Frame rate of scroll.
 *  
 */
void oled_set_horizontal_scroll(scroll_dir_t dir, oled_page_t start, oled_page_t end,
                                    frame_rate_t frame_rate)
{
    uint8_t buffer[7] = {dir, 0x00, start, freq, end, 0x00, 0xFF};
	multicommand_OLED(buffer, 7);
}

/*  \brief  Set screen constrast.
 *
 *  \param  start       1 for start scroll or 0 for stop scrool.
 *  
 */
void oled_set_start_scroll(bool start)
{
    if(start)
        oled_write_command(OLED_START_SCROLL);
    else
       oled_write_command(OLED_STOP_SCROLL); 
}

/*  \brief  Draw pixel on screen.
 *
 *  \param  x       X coordinate of pixel.
 *  \param  y       Y coordinate of pixel.
 *  
 */
void oled_set_pixel(uint8_t x, uint8_t y)
{
    matrix[y/NUM_PAGES][x] |= 0x01 << (y%8);
    oled_write_page(y/NUM_PAGES, matrix[y/NUM_PAGES], PAGE_SIZE);
}

/*  \brief  Clear pixel on screen.
 *
 *  \param  x       X coordinate of pixel.
 *  \param  y       Y coordinate of pixel.
 *  
 */
void oled_reset_pixel(uint8_t x, uint8_t y)
{
    matrix[y/NUM_PAGES][x] = matrix[y/NUM_PAGES][x] ^(matrix[y/NUM_PAGES][x] & (0x01<<y%8));
    oled_write_page(y/NUM_PAGES, matrix[y/NUM_PAGES]);
}

/*  \brief  Clear screen.
 *  
 */
void oled_clear()
{
    for(uint8_t i=0; i<NUM_PAGES; i++)
    {
        for(uint8_t j=0; j<PAGE_SIZE;j++)
          screen[i][j] = 0x00;
    }

    refresh_oled();
}

/*  \brief  Draw string on screen.
 *
 *  \param  str     String for write
 *  \param  x       X coordinate for start write.
 *  \param  y       Y coordinate for start write.
 *  
 */
void oled_draw_string(char *str, uint8_t x, uint8_t y)
{
    uint8_t i=0;

    while(str[i] != '\0')
	{

		for(uint8_t k=0; k<6; k++)
		{
			for(uint8_t j=0; j<8; j++)
			{
                if((font_8_table[str[i] -32][k]>>j & 0x01))
                    matrix[(y+j)/NUM_PAGES][x+k] |= 0x01 << ((y+j)%8);
                else
                    matrix[(y+j)/NUM_PAGES][x+k] = matrix[(y+j)/NUM_PAGES][x+k]^(matrix[(y+j)/NUM_PAGES][x+k] & (0x01<<(y+j)%8));
			}

		}

		x+=6;
		i++;
	}

    refresh_oled();
}

/*  \brief  Draw pixel on screen.
 *
 *  \param bitmap       Pointer tpo bit map of image.
 *  \param  x           X coordinate for start drawing.
 *  \param  y           Y coordinate for start drawing.
 *  \param width        Width of image.
 *  \param size         Size of pointer bitmap.
 *  
 */
void oled_draw_bitmap(const uint8_t bitmap, uint8_t x, uint8_t y, uint8_t width, uint16_t size)
{
    uint8_t j=0;
    uint16_t k=0;
    while(k<size)
    {
        for(uint8_t i=0; i<=width; i++)
        {
            matrix[j][x+i]=reverse(bitmap[k]);
            k++;
        }
        j++;
    }

    refresh_oled(matrix);
}

void oled_draw_rect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    for(uint8_t i=x0; iz x1; i++)
    {
        for(uint8_t j=y0; j<y1; j++)
            matrix[j/NUM_PAGES][i] |= 0x01 << (j%8);
    }
    refresh_oled();
}