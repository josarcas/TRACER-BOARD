/*INCLUDES*****************************************************************************************/
#include "dac.h"

/*FUNCTIONS****************************************************************************************/
void dac_set_value(uint8_t dir, uint16_t value)
{
    uint8_t buffer[2] ={value>>8, value};
    i2c_write_blocking(I2C_PORT, dir, buffer, 2, false);
}
