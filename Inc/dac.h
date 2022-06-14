#ifndef INC_DAC_H
#define INC_DAC_H
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

#define DAC_1_DIR                   0x60
#define DAC_2_DIR                   0x61
#define DAC_MEM_DIR                 0x60

#define PERIOD_US                   40000
#define ELAPCED_US                  200
#define DAC_SIZE_BUFFER             (PERIOD_US/ELAPCED_US) +1

/*PROTOTYPES**************************************************************************************/
void dac_set_value(uint8_t dir, uint16_t value);

#endif