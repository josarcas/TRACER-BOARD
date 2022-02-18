/*
***********************************************************************************************************************
*
*
*
***********************************************************************************************************************
*/

/*INCLUDES************************************************************************************************************/
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"

#include "pico/stdlib.h"

#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/dma.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/*DEFINES***********************************************************************************************************/
//SYSTEM------------------------------------------------------------------------------------------------------------
#define VERSION_APP        1.0

#ifndef SYSTEM_INSTRUCT
#define SYSTEM_INSTRUCT
#define ERROR           0
#define OK              1
#define TRANSMIT        2
#define RECEIVE         3
#define TIME_OUT        4
#define OVERFLOW        5
#define ERROR_RX        6
#define DISCONNECT      7
#endif

#define MAX_SIZE_SYSTEM_QUEUE       10
//UART---------------------------------------------------------------------------------------------------------------
#define UART_PORT                   uart0
#define UART_BAUDRATE               115200
#define UART_PIN_RX                 1
#define UART_PIN_TX                 0
#define MAX_SIZE_BUFFER_RX          50
#define MAX_SIZE_BUFFER_TX          25
#define UART_MAX_TIMEOUT            10

//I2C----------------------------------------------------------------------------------------------------------------
#define I2C_PORT                  i2c0
#define I2C_BAUDRATE              1000000
#define I2C_PIN_SDA               16
#define I2C_PIN_SCL               17
//I2C DAC...........................................................................................................
#define I2C_DIR_1                 0x60
#define I2C_DIR_2                 0x61
#define I2C_DIR_MEM               0x60
//I2C OLED.........................................................................................................
#define I2C_DIR_OLED              60
#define DISPLAY_ON_OLED           0xAF
#define DISPLAY_OFF_OLED          0xAE
#define PAGE_SIZE                 128
#define NUM_PAGES                 8
#define HEIGHT_OLED               64
#define WITDTH_OLED               128

//SPI----------------------------------------------------------------------------------------------------------------
#define SPI_PORT                  spi0
#define SPI_BAUDRATE              500000
#define SPI_PIN_SI                3
#define SPI_PIN_SCL               2
#define SPI_PIN_CS                5

//ADC----------------------------------------------------------------------------------------------------------------
#define ADC_PIN_CH_1               26
#define ADC_PIN_CH_2               27
#define ADC_PIN_CH_3               28
#define ADC_CLK_DIV                0
#define ADC_SIZE_BUFFER            19100

//DAC----------------------------------------------------------------------------------------------------------------
#define PERIOD_US                40000
#define ELAPCED_US               200
#define DAC_SIZE_BUFFER          (PERIOD_US/ELAPCED_US) +1

//DIG POT-----------------------------------------------------------------------------------------------------------
#define POT_SIZE_BUFFER      5

//SYSTEM-------------------------------------------------------------------------------------------------------------
#define LED_STATUS                25
#define RELE_PIN                  15
#define GAIN_PIN                  18
#define OPA_ENA_PIN               21

/*TYPEDEFS*********************************************************************************************************/
typedef enum{
    vce,
    vbe
}curve_t;

typedef struct{
    char cmd;
#ifndef MAX_SIZE_INSTRUCTION_ARG
#define MAX_SIZE_INSTRUCTION_ARG    40
#endif
    char arg[MAX_SIZE_INSTRUCTION_ARG];

}instruction_t;

/*GLOBAL CONST*****************************************************************************************************/
const static uint8_t table_code[96][6] ={
		   {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // sp
		   {0x00, 0x00, 0x00, 0x2f, 0x00, 0x00}, // !
		   {0x00, 0x00, 0x07, 0x00, 0x07, 0x00}, // "
		   {0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14}, // #
		   {0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12}, // $
		   {0x00, 0x23, 0x13, 0x08, 0x64, 0x62}, // %
		   {0x00, 0x36, 0x49, 0x55, 0x22, 0x50}, // &
		   {0x00, 0x00, 0x05, 0x03, 0x00, 0x00}, // '
		   {0x00, 0x00, 0x1c, 0x22, 0x41, 0x00}, // (
		   {0x00, 0x00, 0x41, 0x22, 0x1c, 0x00}, // )
		   {0x00, 0x14, 0x08, 0x3E, 0x08, 0x14}, // *
		   {0x00, 0x08, 0x08, 0x3E, 0x08, 0x08}, // +
		   {0x00, 0x00, 0x00, 0xA0, 0x60, 0x00}, // ,
		   {0x00, 0x08, 0x08, 0x08, 0x08, 0x08}, // -
		   {0x00, 0x00, 0x60, 0x60, 0x00, 0x00}, // .
		   {0x00, 0x20, 0x10, 0x08, 0x04, 0x02}, // /
		   {0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
		   {0x00, 0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
		   {0x00, 0x42, 0x61, 0x51, 0x49, 0x46}, // 2
		   {0x00, 0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
		   {0x00, 0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
		   {0x00, 0x27, 0x45, 0x45, 0x45, 0x39}, // 5
		   {0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
		   {0x00, 0x01, 0x71, 0x09, 0x05, 0x03}, // 7
		   {0x00, 0x36, 0x49, 0x49, 0x49, 0x36}, // 8
		   {0x00, 0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
		   {0x00, 0x00, 0x36, 0x36, 0x00, 0x00}, // :
		   {0x00, 0x00, 0x56, 0x36, 0x00, 0x00}, // ;
		   {0x00, 0x08, 0x14, 0x22, 0x41, 0x00}, // <
		   {0x00, 0x14, 0x14, 0x14, 0x14, 0x14}, // =
		   {0x00, 0x00, 0x41, 0x22, 0x14, 0x08}, // >
		   {0x00, 0x02, 0x01, 0x51, 0x09, 0x06}, // ?
		   {0x00, 0x32, 0x49, 0x59, 0x51, 0x3E}, // @
		   {0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C}, // A
		   {0x00, 0x7F, 0x49, 0x49, 0x49, 0x36}, // B
		   {0x00, 0x3E, 0x41, 0x41, 0x41, 0x22}, // C
		   {0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
		   {0x00, 0x7F, 0x49, 0x49, 0x49, 0x41}, // E
		   {0x00, 0x7F, 0x09, 0x09, 0x09, 0x01}, // F
		   {0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
		   {0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
		   {0x00, 0x00, 0x41, 0x7F, 0x41, 0x00}, // I
		   {0x00, 0x20, 0x40, 0x41, 0x3F, 0x01}, // J
		   {0x00, 0x7F, 0x08, 0x14, 0x22, 0x41}, // K
		   {0x00, 0x7F, 0x40, 0x40, 0x40, 0x40}, // L
		   {0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
		   {0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
		   {0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
		   {0x00, 0x7F, 0x09, 0x09, 0x09, 0x06}, // P
		   {0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
		   {0x00, 0x7F, 0x09, 0x19, 0x29, 0x46}, // R
		   {0x00, 0x46, 0x49, 0x49, 0x49, 0x31}, // S
		   {0x00, 0x01, 0x01, 0x7F, 0x01, 0x01}, // T
		   {0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
		   {0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
		   {0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
		   {0x00, 0x63, 0x14, 0x08, 0x14, 0x63}, // X
		   {0x00, 0x07, 0x08, 0x70, 0x08, 0x07}, // Y
		   {0x00, 0x61, 0x51, 0x49, 0x45, 0x43}, // Z
		   {0x00, 0x00, 0x7F, 0x41, 0x41, 0x00}, // [
		   {0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55}, // 55
		   {0x00, 0x00, 0x41, 0x41, 0x7F, 0x00}, // ]
		   {0x00, 0x04, 0x02, 0x01, 0x02, 0x04}, // ^
		   {0x00, 0x40, 0x40, 0x40, 0x40, 0x40}, // _
		   {0x00, 0x00, 0x01, 0x02, 0x04, 0x00}, // '
		   {0x00, 0x20, 0x54, 0x54, 0x54, 0x78}, // a
		   {0x00, 0x7F, 0x48, 0x44, 0x44, 0x38}, // b
		   {0x00, 0x38, 0x44, 0x44, 0x44, 0x20}, // c
		   {0x00, 0x38, 0x44, 0x44, 0x48, 0x7F}, // d
		   {0x00, 0x38, 0x54, 0x54, 0x54, 0x18}, // e
		   {0x00, 0x08, 0x7E, 0x09, 0x01, 0x02}, // f
		   {0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C}, // g
		   {0x00, 0x7F, 0x08, 0x04, 0x04, 0x78}, // h
		   {0x00, 0x00, 0x44, 0x7D, 0x40, 0x00}, // i
		   {0x00, 0x40, 0x80, 0x84, 0x7D, 0x00}, // j
		   {0x00, 0x7F, 0x10, 0x28, 0x44, 0x00}, // k
		   {0x00, 0x00, 0x41, 0x7F, 0x40, 0x00}, // l
		   {0x00, 0x7C, 0x04, 0x18, 0x04, 0x78}, // m
		   {0x00, 0x7C, 0x08, 0x04, 0x04, 0x78}, // n
		   {0x00, 0x38, 0x44, 0x44, 0x44, 0x38}, // o
		   {0x00, 0xFC, 0x24, 0x24, 0x24, 0x18}, // p
		   {0x00, 0x18, 0x24, 0x24, 0x18, 0xFC}, // q
		   {0x00, 0x7C, 0x08, 0x04, 0x04, 0x08}, // r
		   {0x00, 0x48, 0x54, 0x54, 0x54, 0x20}, // s
		   {0x00, 0x04, 0x3F, 0x44, 0x40, 0x20}, // t
		   {0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
		   {0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
		   {0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
		   {0x00, 0x44, 0x28, 0x10, 0x28, 0x44}, // x
		   {0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C}, // y
		   {0x00, 0x44, 0x64, 0x54, 0x4C, 0x44}, // z
		   {0x00, 0x00, 0x08, 0x77, 0x00, 0x00}, // {
		   {0x00, 0x00, 0x00, 0x7F, 0x00, 0x00}, // |
		   {0x00, 0x00, 0x77, 0x08, 0x00, 0x00}, // }
		   {0x00, 0x10, 0x08, 0x10, 0x08, 0x00}, // ~
		   {0x14, 0x14, 0x14, 0x14, 0x14, 0x14}  // horiz lines
		 };

const static uint8_t esimeico[518] =  { 0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X01,0X01,0X02,0X01,0X00,0X00,0X00,0X00,0X00,0X07,0X04,
0X04,0X06,0X00,0X00,0X00,0X00,0X00,0X01,0X02,0X00,0X01,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X16,0X11,
0X10,0X11,0X09,0X03,0X03,0XC3,0X1D,0X01,0X03,0X83,0X33,0X06,0X16,0X17,0X27,0X07,
0X07,0X06,0X96,0X17,0X07,0X33,0XC3,0X03,0X01,0X1D,0XE2,0X02,0X03,0X0D,0X09,0X10,
0X01,0X12,0X08,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X03,0X0F,0X3F,0XFE,0XFE,0XFE,0XEE,0XEE,0XEE,0XEF,0XFE,0X10,0X13,
0XAF,0XFF,0XFF,0XFF,0XEF,0XEF,0XEE,0XEF,0XEF,0XFF,0XEF,0X20,0X00,0XFF,0XFF,0XFF,
0XFF,0X00,0X21,0XFF,0XFF,0XFF,0XFF,0X7F,0X9F,0X9F,0XFF,0XFF,0XFF,0XFF,0XFF,0XBF,
0X13,0X01,0XFF,0XFF,0XFF,0XEE,0XEE,0XFE,0XEE,0XEE,0X6E,0X0E,0X02,0X00,0X00,0X00,
0X00,0X00,0X70,0XF0,0XF0,0XF8,0X74,0X74,0X72,0X72,0X72,0X73,0XF0,0XFF,0XFC,0XC0,
0X00,0XF3,0XF0,0XF0,0XF0,0XF8,0XF0,0XF3,0XFF,0XFF,0XFE,0XBC,0X3F,0XF6,0XF3,0XF1,
0XF1,0X63,0X67,0XFE,0XFC,0XFC,0X3F,0XCF,0XE3,0XF0,0XC8,0X80,0XF0,0XF0,0XF7,0XF0,
0XC0,0XF8,0XFF,0XF3,0XF3,0X72,0X72,0X72,0X74,0X74,0X78,0X70,0X70,0X70,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X1C,0X00,0X22,0XA2,0X02,0X40,0XE4,0X33,0XFC,0X3F,0X0F,
0X0E,0XC7,0X0B,0X03,0X09,0X07,0X17,0XF3,0XFF,0XFF,0X78,0X0F,0XC7,0X3E,0X3B,0XE3,
0XE3,0X33,0X1E,0X4F,0X0F,0X39,0XF7,0XFF,0XF3,0X13,0X07,0X09,0X03,0X0B,0XE7,0X06,
0X0D,0X1F,0XFC,0XF3,0XE4,0X40,0X42,0XA2,0X22,0X22,0X1C,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0C,0X12,0X36,0X56,0X54,0X14,0X94,0XD0,
0X73,0X32,0X12,0X90,0XD1,0XF0,0XE8,0XF4,0XFB,0XFF,0XFC,0XFC,0XBF,0X9F,0X1E,0X0E,
0X0E,0X1E,0X9F,0XBB,0XFE,0XFC,0XFF,0XFF,0XF4,0XE8,0XF0,0XD0,0X90,0X12,0X32,0X73,
0XF0,0X90,0X14,0XD4,0X56,0X36,0X32,0X1C,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X68,0X84,
0X00,0X08,0X10,0X60,0X40,0X83,0X68,0X7E,0XFF,0X43,0XEF,0XE9,0X69,0XEB,0X76,0X3C,
0X1C,0X76,0XEB,0XA9,0XE9,0XED,0X63,0XFF,0X7F,0X78,0X23,0X40,0X60,0X10,0X08,0X00,
0X84,0X68,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X80,0X40,0X40,0XC0,0X80,0XC0,0X80,0X80,0X00,0X60,0X00,
0X00,0X60,0X00,0X80,0X80,0XC0,0X80,0XC0,0X40,0X40,0X80,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
};

const static float IB_MEASSURE_VALUES[]={
    766.798419,    383.3992095,    343.8735178,    300.3952569,    268.7747036,    233.201581,    213.4387352,
    196.0474308,    180.2371542,    166.0079051,    154.1501976,    142.2924901,    133.5968379,    124.9011858,
    117.7865613,    110.6719368,    105.1383399,    100.3952569,    94.86166008,    90.90909091,    86.95652174,
    83.39920949,    79.84189723,    76.6798419,    73.51778656,    71.14624506,    68.77470356,    66.40316206,
    64.03162055,    62.05533597,    60.07905138,    58.1027668,    56.52173913,    54.94071146,    53.55731225,
    52.17391304,    50.59288538,    49.40711462,    48.22134387,    47.03557312,    45.84980237,    44.46640316,
    43.47826087,    42.68774704,    41.69960474,    40.90909091,    39.92094862,    39.13043478,    38.33992095,
    37.74703557,    37.1541502,    36.36363636,    35.77075099,    35.17786561,    34.38735178,    33.99209486,
    33.39920949,    32.60869565,    32.21343874,    31.62055336,    31.22529644,    30.63241107,    30.23715415,
    29.64426877,    29.24901186,    29.0513834,    28.45849802,    27.86561265,    27.47035573,    27.07509881,
    26.87747036,    26.28458498,    26.08695652,    25.6916996,    25.49407115,    25.09881423,    24.70355731,
    24.3083004,    24.11067194,    23.91304348,    23.275,    22.98765432,    22.70731707,    22.43373494,
    22.16666667,    21.90588235,    21.65116279,    21.40229885,    21.15909091,    20.92134831,    20.68888889,
    20.46153846,    20.23913043,    20.02150538,    19.80851064,    19.6,    19.39583333,    19.19587629,
    19,    18.80808081,    18.62,    18.43564356,    18.25490196,    18.0776699,    17.90384615,    17.73333333,
    17.56603774,    17.40186916,    17.24074074,    17.08256881,    16.92727273,    16.77477477,    16.625,
    16.47787611,    16.33333333,    16.19130435,    16.05172414,    15.91452991,    15.77966102,    15.64705882,
    15.51666667,    15.38842975,    15.26229508,    15.13821138,    15.01612903,    14.896,    14.77777778,
    14.66141732,    14.546875,    14.43410853,    14.32307692,    14.21374046,    14.10606061,    14,
    13.89552239,    13.79259259,    13.69117647,    13.59124088,    13.49275362,    13.39568345,    13.3,
    13.20567376,    13.11267606,    13.02097902,    12.93055556,    12.84137931,    12.75342466,    12.66666667,
    12.58108108,    12.4966443,    12.41333333,    12.33112583,    12.25,    12.16993464,    12.09090909,
    12.01290323,    11.93589744,    11.85987261,    11.78481013,    11.71069182,    11.6375,    11.56521739,
    11.49382716,    11.42331288,    11.35365854,    11.28484848,    11.21686747,    11.1497006,    11.08333333,
    11.01775148,    10.95294118,    10.88888889,    10.8255814,    10.76300578,    10.70114943,    10.64,
    10.57954545,    10.51977401,    10.46067416,    10.40223464,    10.34444444,    10.28729282,    10.23076923,
    10.17486339,    10.11956522,    10.06486486,    10.01075269,    9.957219251,    9.904255319,    9.851851852,
    9.8,    9.748691099,    9.697916667,    9.647668394,    9.597938144,    9.548717949,    9.5,    9.45177665,
    9.404040404,    9.35678392,    9.31,    9.263681592,    9.217821782,    9.172413793,    9.12745098,    9.082926829,
    9.038834951,    8.995169082,    8.951923077,    8.909090909,    8.866666667,    8.82464455,    8.783018868,
    8.741784038,    8.700934579,    8.660465116,    8.62037037,    8.580645161,    8.541284404,    8.502283105,
    8.463636364,    8.425339367,    8.387387387,    8.349775785,    8.3125,    8.275555556,    8.238938053,
    8.202643172,    8.166666667,    8.131004367,    8.095652174,    8.060606061,    8.025862069,    7.991416309,
    7.957264957,    7.923404255,    7.889830508,    7.856540084,    7.823529412,    7.790794979,    7.758333333,
    7.726141079,    7.694214876,    7.66255144,    7.631147541,    7.6,    7.569105691,    7.538461538,    7.508064516,
    7.477911647,    7.448,    7.418326693,    7.388888889,    7.359683794,    7.330708661,    7.301960784,
    };





/*GLOBAL VARIABLES*************************************************************************************************/
//SYSTEM-----------------------------------------------------------------------------------------------------------
curve_t type;
uint8_t n_samples;

//UART-------------------------------------------------------------------------------------------------------------
//char buffer_rx[MAX_SIZE_BUFFER_RX];
//char buffer_tx[MAX_SIZE_BUFFER_TX];
//int uart_irq;

//ADC--------------------------------------------------------------------------------------------------------------
uint16_t adc[ADC_SIZE_BUFFER];
//DAC--------------------------------------------------------------------------------------------------------------
uint16_t index_dac;
uint16_t dac_values[DAC_SIZE_BUFFER];
float amp_ch1;
repeating_timer_t timer;

//DIG POT----------------------------------------------------------------------------------------------------------
uint8_t resistor_value;

//DMA-------------------------------------------------------------------------------------------------------------
uint dma_ch;
dma_channel_config dma_config;

//OLED------------------------------------------------------------------------------------------------------------
uint8_t screen[NUM_PAGES][PAGE_SIZE];

//SEMAPHORES------------------------------------------------------------------------------------------------------
//SemaphoreHandle_t serial_semphr = NULL;
SemaphoreHandle_t qt_comprobe_con = NULL;
SemaphoreHandle_t end_probe_semphr = NULL;

//MUTEX-----------------------------------------------------------------------------------------------------------
SemaphoreHandle_t serial_mutex = NULL;

//QUEUE-----------------------------------------------------------------------------------------------------------
QueueHandle_t status_queue = NULL;
QueueHandle_t app_instruction_queue = NULL;

/*PROTOTYPES*******************************************************************************************************/
//SYSTEM-----------------------------------------------------------------------------------------------------------
void delay_cycles(uint32_t cycles);
void set_opa(bool ena);
bool i2c_check_response(uint8_t dir, uint32_t timeout);
void debug(const char *format, ...);

bool start_probe();
void enable_opa(bool ena);
void set_rele(bool state);

void init_digital_outputs();

//UART-------------------------------------------------------------------------------------------------------------
void init_serial();
void interrupt_serial();
void transmit_serial(char *message);
bool read_instruct(instruction_t *qt_instruct, char *instruct);
uint8_t transmit_adc_values();

//I2C--------------------------------------------------------------------------------------------------------------
void init_i2c_bus();
void check_i2c_devices();

//ADC--------------------------------------------------------------------------------------------------------------
void init_adc();

//DAC--------------------------------------------------------------------------------------------------------------
void init_dac();
void set_dac_value(uint8_t dir, uint16_t value);
void generate_ramp();
bool timer1Callback(repeating_timer_t *timer);
bool timer2Callback(repeating_timer_t *timer);

//DIG POT----------------------------------------------------------------------------------------------------------
void init_dig_pot();
void set_dig_pot(uint8_t value);

//DMA--------------------------------------------------------------------------------------------------------------
void init_dma();

//TASK------------------------------------------------------------------------------------------------------------
void system_status_task(void *arg);
void comprobe_connection_task(void *arg);
void serial_receive_task(void *arg);
void app_main_task(void *arg);

//OLED------------------------------------------------------------------------------------------------------------
void write_command_oled(uint8_t cmd);
void write_data_oled(uint8_t data);
void init_oled();
void write_page_oled(uint8_t page, uint8_t *data, uint16_t len);
void set_pixel_oled(uint8_t **matrix, uint8_t x, uint8_t y);
void reset_pixel_oled(uint8_t **matrix, uint8_t x, uint8_t y);
void refresh_oled(uint8_t matrix[NUM_PAGES][PAGE_SIZE]);
void draw_string_oled(char *str, uint8_t matrix[NUM_PAGES][PAGE_SIZE], uint8_t x, uint8_t y);
void draw_char_oled(char str, uint8_t matrix[NUM_PAGES][PAGE_SIZE], uint8_t x, uint8_t y);
void clear_oled(uint8_t matrix[NUM_PAGES][PAGE_SIZE]);
void draw_bitmap_oled(uint8_t matrix[NUM_PAGES][PAGE_SIZE], const uint8_t *bitmap, uint16_t size, uint8_t weigth, uint8_t x, uint8_t y);
void init_screen();

/*---------------------------------------------MAIN FUNCTIION----------------------------------------------------*/
int main()
{
    //SYSTEM INIT-------------------------------------------------------------------------------------------------
    stdio_init_all();
    set_sys_clock_khz(130000, true);

    init_digital_outputs();

    init_serial();
    init_i2c_bus();
    init_screen();
    check_i2c_devices();
    init_dac();
    init_adc();
    init_dig_pot();
    init_dma();

    //CREATE SEMAPHORES-------------------------------------------------------------------------------------------
    //serial_semphr = xSemaphoreCreateBinary();
    qt_comprobe_con = xSemaphoreCreateBinary();
    end_probe_semphr = xSemaphoreCreateBinary();

    //CREATE MUTEX------------------------------------------------------------------------------------------------
    serial_mutex = xSemaphoreCreateMutex();

    //CREATE QUEUE-----------------------------------------------------------------------------------------------
    status_queue = xQueueCreate(MAX_SIZE_SYSTEM_QUEUE, sizeof(uint8_t));
    app_instruction_queue = xQueueCreate(2, sizeof(instruction_t));

    //CREATE TASK------------------------------------------------------------------------------------------------
    xTaskCreate(&system_status_task, "system status", 1024, NULL, 1, NULL);
    xTaskCreate(&comprobe_connection_task, "comprobe con", 1024, NULL, 1, NULL);
    xTaskCreate(&serial_receive_task, "serial rx", 1024*2, NULL, 3, NULL);
    xTaskCreate(&app_main_task, "main app", 1024*2, NULL, 2, NULL);

    //TASK START-------------------------------------------------------------------------------------------------
    vTaskStartScheduler();

}

/*FUNCTIONS*******************************************************************************************************/
//SYSTEM-----------------------------------------------------------------------------------------------------------
void delay_cycles(uint32_t cycles)
{
    while (cycles --> 0);    
}

void set_opa(bool ena)
{
    gpio_put(OPA_ENA_PIN, ena);
    delay_cycles(100);
}

bool i2c_check_response(uint8_t dir, uint32_t timeout)
{
    uint8_t buffer;
    if(i2c_read_timeout_us(I2C_PORT, dir, &buffer, 1, true, timeout)>0)
        return true;
    
    return false;
}

void debug(const char *format, ...)
{
    va_list args;
    static uint8_t x=0, y=0;
    char buffer[5];
    char aux;

    va_start(args, format);
 
    while (*format != '\0') {
        if(*format == '%')
        {
            format++;
            switch (*format)
            {
                case 'd': 
                {
                    int i = va_arg(args, int);
                    sprintf(buffer, "%d", i);
                    draw_string_oled(buffer, screen, x, y);
                    x+= (6*strlen(buffer));
                } 
                break;

                case 'c': 
                {
                    int c = va_arg(args, int);
                    draw_char_oled(c, screen, x, y);
                    x+=6; 
                } 
                break;

                case 'f':
                {
                    double d = va_arg(args, double);
                    sprintf(buffer, "%.2f", d);
                    draw_string_oled(buffer, screen, x, y);
                    x+= (6*strlen(buffer));
                }
                break;
                default:
                break;
            }

        }

        else if(*format == '\n')
        {
            y+=8;
            x=0;
        }

        else if(*format == '\t')
        {
            while(x<WITDTH_OLED)
            {
                draw_char_oled(' ', screen, x, y);
                x+=6;
            }
            x=0;
        }

        else
        {
            aux = *format;
            draw_char_oled(aux, screen , x, y);
            x+=6;
        }

        ++format;
        if(x>WITDTH_OLED)
        {
            x=0;
            y+=8;
        }

        if(y>= HEIGHT_OLED)
        {
            y=0;
            x=0;
        }
    }
 
    va_end(args);
}


bool start_probe()
{
    index_dac = 0;
    enable_opa(true);

    if(type == vce)
    {
        set_rele(false);
        adc_set_round_robin(0x01<<(ADC_PIN_CH_1-26)|0x01<<(ADC_PIN_CH_2-26));        
        return add_repeating_timer_us(-ELAPCED_US, timer1Callback, NULL, &timer);
    }

    else
    {
        set_rele(true);
        adc_set_round_robin(0x01<<(ADC_PIN_CH_2-26)|0x01<<(ADC_PIN_CH_3-26));
        set_dac_value(I2C_DIR_1, 0);
        delay_cycles(100);
        return add_repeating_timer_us(-ELAPCED_US, timer2Callback, NULL, &timer);
    }


}

void enable_opa(bool ena)
{
    gpio_put(OPA_ENA_PIN, ena);
    delay_cycles(1000);
}

void set_rele(bool state)
{
    delay_cycles(1000000);
    gpio_put(RELE_PIN, state);
    delay_cycles(1000000);
    gpio_put(GAIN_PIN, state);
}

void init_digital_outputs()
{
    gpio_init(LED_STATUS);
    gpio_set_dir(LED_STATUS, GPIO_OUT);

    gpio_init(RELE_PIN);
    gpio_set_dir(RELE_PIN, GPIO_OUT);

    gpio_init(OPA_ENA_PIN);
    gpio_set_dir(OPA_ENA_PIN, GPIO_OUT);

    gpio_init(GAIN_PIN);
    gpio_set_dir(GAIN_PIN, GPIO_OUT);
}

//UART-------------------------------------------------------------------------------------------------------------
void init_serial()
{
    uart_init(UART_PORT, UART_BAUDRATE);
    gpio_set_function(UART_PIN_RX, GPIO_FUNC_UART);
    gpio_set_function(UART_PIN_TX, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_PORT, false, false);
    uart_set_format(UART_PORT, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_PORT, true);
}


void transmit_serial(char *message)
{
    char buffer_tx[MAX_SIZE_BUFFER_TX];
    sprintf(buffer_tx, "RP:%d;%send", strlen(message)+3, message);


    if(xSemaphoreTake(serial_mutex, portMAX_DELAY) == pdTRUE)
    {
        uart_puts(UART_PORT, buffer_tx);
        xSemaphoreGive(serial_mutex);
    }
}

bool read_instruct(instruction_t *qt_instruct, char *instruct)
{
    char *token;
    int size;

    if(instruct[0] != 'Q')
        return false;
    if(instruct[1] != 'T')
        return false;
    if(instruct[2] != ':')
        return false;

    char aux[MAX_SIZE_BUFFER_RX];

    strcpy(aux, instruct);    

    token = strtok(aux, ":");
    token = strtok(NULL, ";");
    size = atoi(token);
    token = strtok(NULL, ".");

    if(size != strlen(token))
        return false;

    size = 0;

    while(instruct[size] != ';')
        size++;
    
    size++;
    qt_instruct->cmd = instruct[size];
    size+=2;

    for(uint8_t i=0; i<MAX_SIZE_INSTRUCTION_ARG; i++)
    {
        qt_instruct->arg[i] = instruct[size];
        size++; 
        if(instruct[size] == '.')
            break;
    }

    return true;
}

uint8_t transmit_adc_values()
{
    char buffer[MAX_SIZE_BUFFER_TX];
    char curve_type;

    if(type == vce)
        curve_type = 'e';
    else
        curve_type = 'f';

    int size = (ADC_SIZE_BUFFER*1.5)+5;

    sprintf(buffer, "RP:%d;%c,", size, curve_type);

    if(xSemaphoreTake(serial_mutex, portMAX_DELAY) == pdTRUE)
    {
        uart_puts(UART_PORT, buffer);

        for(uint16_t i=0; i<ADC_SIZE_BUFFER; i+=2)
        {
            uart_putc(UART_PORT, adc[i]);
            uart_putc(UART_PORT, (adc[i]>>4&0xF0)|adc[i+1]>>8);
            uart_putc(UART_PORT, adc[i+1]);

        }

        uart_puts(UART_PORT, "end");

        xSemaphoreGive(serial_mutex);

        return TRANSMIT;
    }

    return ERROR;
}

//I2C--------------------------------------------------------------------------------------------------------------
void init_i2c_bus()
{
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_PIN_SCL, GPIO_FUNC_I2C);
    gpio_set_function(I2C_PIN_SDA, GPIO_FUNC_I2C);
}

void check_i2c_devices()
{
    if(!i2c_check_response(I2C_DIR_1, 1000))
    {
        debug("DAC1 not response\n");
        while(1);        
    }
    if(!i2c_check_response(I2C_DIR_2, 1000))
    {
        debug("DAC2 not response\n");
        while(1);
    }

}

//ADC--------------------------------------------------------------------------------------------------------------
void init_adc()
{
    adc_gpio_init(ADC_PIN_CH_1);
    adc_gpio_init(ADC_PIN_CH_2);
    adc_gpio_init(ADC_PIN_CH_3);
    adc_set_clkdiv(ADC_CLK_DIV);
    adc_init();
    adc_fifo_setup(true, true, 1, true, false);
}

//DAC--------------------------------------------------------------------------------------------------------------
void init_dac()
{
    set_dac_value(I2C_DIR_1, 0);
    set_dac_value(I2C_DIR_2, 0);
}

void set_dac_value(uint8_t dir, uint16_t value)
{
    uint8_t buffer[2] ={value>>8, value};
    i2c_write_blocking(I2C_PORT, dir, buffer, 2, false); 
}

void generate_ramp()
{
    float time = 0;

    if(type == vce)
    {
        amp_ch1 = (float)amp_ch1/(float)10;
        for(uint16_t i=0; i<DAC_SIZE_BUFFER; i++)
        {
            dac_values[i] = (((float)amp_ch1/(float)PERIOD_US)*time)*4095;
            time += ELAPCED_US;
        }
    }

    else
    {
        for(uint16_t i=0; i<DAC_SIZE_BUFFER; i++)
        {
            dac_values[i] = (((float)0.3030/(float)PERIOD_US)*time)*4095;
            time += ELAPCED_US;
        }

    }
}

//DIG POT----------------------------------------------------------------------------------------------------------
void init_dig_pot()
{
    spi_init(SPI_PORT, SPI_BAUDRATE);
    gpio_set_function(SPI_PIN_SI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_PIN_SCL, GPIO_FUNC_SPI);
    gpio_init(SPI_PIN_CS);
    gpio_set_dir(SPI_PIN_CS, GPIO_OUT);

    spi_set_slave(SPI_PORT, false);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_put(SPI_PIN_CS, 1);

    set_dig_pot(255); 
}

void set_dig_pot(uint8_t value)
{
    uint8_t buffer[2]={0x11, value};
    gpio_put(SPI_PIN_CS, 0);
    spi_write_blocking(SPI_PORT, buffer, 2);
    gpio_put(SPI_PIN_CS, 1);
}

//DMA--------------------------------------------------------------------------------------------------------------
void init_dma()
{
    dma_ch = dma_claim_unused_channel(true);
    dma_config = dma_channel_get_default_config(dma_ch);

    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_config, false);
    channel_config_set_write_increment(&dma_config, true);
    channel_config_set_dreq(&dma_config, DREQ_ADC);
}

//TASK------------------------------------------------------------------------------------------------------------
void system_status_task(void *arg)
{
    uint8_t status;

    clear_oled(screen);

    while(1)
    {
        if(xQueueReceive(status_queue, &status, portMAX_DELAY) == pdTRUE)
        {

            switch (status)
            {
            case ERROR:
            {
                transmit_serial("c,Error");
                for(uint8_t i=0; i<10; i++)
                {
                    gpio_put(LED_STATUS, 0);
                    vTaskDelay(10);
                    gpio_put(LED_STATUS, 1);
                    vTaskDelay(10);
                }
            }
                break;

            case OK:
            {
                gpio_put(LED_STATUS, 1);
            }
                break;
            
            case TRANSMIT:
            {
                //draw_string_oled("Data transmit", screen, 0, i);
            }
                break;

            case RECEIVE:
            {
                //draw_string_oled("Data receive", screen, 0, i);
            }
                break;

            case TIME_OUT:
            {
                transmit_serial("c,Timeout");
                debug("Error Timeout\t\n");
            }
                break;

            case OVERFLOW:
            {
                transmit_serial("c,Overflow");
                debug("Buffer overflow\t\n");

            }
                break;
            
            case ERROR_RX:
            {
                transmit_serial("c,ErrorReceive");
                debug("Error reception\t\n");
            }
                break;

            case DISCONNECT:
            {
                debug("Disconnect\t\n");
            }
                break;
            
            default:
                break;
            }
        }
    }
}

void comprobe_connection_task(void *arg)
{
    uint8_t status;
    while(1)
    {
        transmit_serial("c,0");

        if(xSemaphoreTake(qt_comprobe_con, 100) != pdTRUE)
        {
            status = DISCONNECT;
            xQueueSend(status_queue, &status, portMAX_DELAY);
        }
        vTaskDelay(1000);
    }
}

void serial_receive_task(void *arg)
{
    uint8_t index;
    TickType_t timeout;
    uint8_t status;
    char buffer_rx[MAX_SIZE_BUFFER_RX];
    instruction_t qt_instruct;

    while(1)
    {
        if(uart_is_readable(UART_PORT))
        {
            index = 0;
            memset(buffer_rx, '\0', MAX_SIZE_BUFFER_RX);
            timeout = xTaskGetTickCount() + UART_MAX_TIMEOUT;

            while(1)
            {
                if(uart_is_readable(UART_PORT))
                {
                    buffer_rx[index] = uart_getc(UART_PORT);
                    index++;
                }

                if(buffer_rx[index-1] == '.')
                {
                    status = RECEIVE;
                    break;
                }

                if(index>=MAX_SIZE_BUFFER_RX)
                {
                    status = OVERFLOW;
                    break;
                }

                if(xTaskGetTickCount()>timeout)
                {
                    status = TIME_OUT;
                    break;
                }
            } 
        
            xQueueSend(status_queue, &status, portMAX_DELAY);

            if(status == RECEIVE)
            {
                if(read_instruct(&qt_instruct, buffer_rx))
                    xQueueSend(app_instruction_queue, &qt_instruct, portMAX_DELAY);

                else
                {
                    status = ERROR_RX;
                    xQueueSend(status_queue, &status, portMAX_DELAY);
                }
            }
            
        }
        vTaskDelay(1);
    }
}

void app_main_task(void *arg)
{
    instruction_t qt_instruct;
    uint8_t status;
    char *token;

    while(1)
    {
        if(xQueueReceive(app_instruction_queue, &qt_instruct, 10) == pdTRUE)
        {
            switch (qt_instruct.cmd)
            {
                case '0':
                    xSemaphoreGive(qt_comprobe_con);
                    break;

                case 'a':
                    type = vce;
                    token = strtok(qt_instruct.arg, "-");
                    amp_ch1 = (float)atoi(token)/(float)10;
                    token = strtok(NULL, "-");
                    resistor_value = atoi(token);
                    token = strtok(NULL, "-");
                    n_samples = atoi(token);
                    debug("\n\tVCE:%f\nIB:%f\t\n", amp_ch1, IB_MEASSURE_VALUES[resistor_value], n_samples);
                    generate_ramp();             
                    break;

                case 'b':
                    type = vbe;
                    generate_ramp(); 
                    break;

                case 'c':
                    debug("Starting test\t\n");
                    if(!start_probe())
                        debug("Error test\t\n");
                    break;
                
                default:
                    break;
            }
        }

        if(xSemaphoreTake(end_probe_semphr, 10) == pdTRUE)
        {
            status = transmit_adc_values();
            if(status == TRANSMIT)
                debug("Transmit\t\n");

            xQueueSend(status_queue, &status, portMAX_DELAY);
        }
    }

}

//TIMER-----------------------------------------------------------------------------------------------------------
bool timer1Callback(repeating_timer_t *timer)
{
    
    set_dac_value(I2C_DIR_1, dac_values[index_dac]);
    if(index_dac == 0)
    {
        adc_run(false);
        adc_fifo_drain();
        hw_write_masked(&adc_hw->cs, 0 << ADC_CS_AINSEL_LSB, ADC_CS_AINSEL_BITS);
        set_dig_pot(resistor_value);
        delay_cycles(50);
        dma_channel_configure(dma_ch, &dma_config, adc, &adc_hw->fifo, ADC_SIZE_BUFFER, true);
        adc_run(true);
    }
    index_dac++;

    if(index_dac>=DAC_SIZE_BUFFER)
    {
        adc_run(false);
        adc_fifo_drain();
        set_dac_value(I2C_DIR_1, 0);
        set_opa(false);
        set_dig_pot(255);
        xSemaphoreGiveFromISR(end_probe_semphr, pdFALSE);
        return false;
    }

    return true;
}

bool timer2Callback(repeating_timer_t *timer)
{
    set_dac_value(I2C_DIR_2, dac_values[index_dac]);

    if(index_dac == 0)
    {
        adc_run(false);
        adc_fifo_drain();
        hw_write_masked(&adc_hw->cs, 0 << ADC_CS_AINSEL_LSB, ADC_CS_AINSEL_BITS);
        delay_cycles(50);
        dma_channel_configure(dma_ch, &dma_config, adc, &adc_hw->fifo, ADC_SIZE_BUFFER, true);
        adc_run(true);
    }

    index_dac++;

    if(index_dac>=DAC_SIZE_BUFFER)
    {
        adc_run(false);
        adc_fifo_drain();
        set_dac_value(I2C_DIR_1, 0);
        set_dac_value(I2C_DIR_2, 0);
        set_opa(false);
        xSemaphoreGiveFromISR(end_probe_semphr, pdFALSE);
        return false;
    }

    return true;
    
}

//OLED------------------------------------------------------------------------------------------------------------
void write_command_oled(uint8_t cmd)
{
    uint8_t buffer[2] = {0x00, cmd};

    i2c_write_blocking(I2C_PORT, I2C_DIR_OLED, buffer, 2, false);
}

void write_data_oled(uint8_t data)
{
    uint8_t buffer[2]={0x40, data};

    i2c_write_blocking(I2C_PORT, I2C_DIR_OLED, buffer, 2, false);
}

void init_oled()
{
	write_command_oled(0xa8);
	write_command_oled(0x3f);
	write_command_oled(0xd3);
	write_command_oled(0x00);
	write_command_oled(0x40);
	write_command_oled(0xa1);
	write_command_oled(0xc8);
	write_command_oled(0xad);
	write_command_oled(0x02);
	write_command_oled(0x81);
	write_command_oled(0x7f);
	write_command_oled(0xa4);
	write_command_oled(0xa6);
	write_command_oled(0xd5);
	write_command_oled(0x80);
	write_command_oled(0x8d);
	write_command_oled(0x14);
	write_command_oled(0xaf);

}

void write_page_oled(uint8_t page, uint8_t *data, uint16_t len)
{
    uint8_t buffer[len+1];

    buffer[0] = 0x40;
	for(uint8_t i=0; i<128; i++)
		buffer[i+1] = data[i];

	write_command_oled(0xb0 + page);
	write_command_oled(0x00);
	write_command_oled(0x10);

    i2c_write_blocking(I2C_PORT, I2C_DIR_OLED, buffer, len+1, false);
}

void set_pixel_oled(uint8_t **matrix, uint8_t x, uint8_t y)
{
    matrix[y/NUM_PAGES][x] |= 0x01 << (y%8);
    write_page_oled(y/NUM_PAGES, matrix[y/NUM_PAGES], PAGE_SIZE);
}

void reset_pixel_oled(uint8_t **matrix, uint8_t x, uint8_t y)
{
    matrix[y/NUM_PAGES][x] = matrix[y/NUM_PAGES][x] ^(matrix[y/NUM_PAGES][x] & (0x01<<y%8));
    write_page_oled(y/NUM_PAGES, matrix[y/NUM_PAGES], PAGE_SIZE);
}

void refresh_oled(uint8_t matrix[NUM_PAGES][PAGE_SIZE])
{
    for(uint16_t i=0; i<NUM_PAGES; i++)
        write_page_oled(i, matrix[i], PAGE_SIZE);
}

void draw_string_oled(char *str, uint8_t matrix[NUM_PAGES][PAGE_SIZE], uint8_t x, uint8_t y)
{
    uint8_t i=0;

    while(str[i] != '\0')
	{

		for(uint8_t k=0; k<6; k++)
		{
			for(uint8_t j=0; j<8; j++)
			{
                if((table_code[str[i] -32][k]>>j & 0x01))
                    matrix[(y+j)/NUM_PAGES][x+k] |= 0x01 << ((y+j)%8);
                else
                    matrix[(y+j)/NUM_PAGES][x+k] = matrix[(y+j)/NUM_PAGES][x+k]^(matrix[(y+j)/NUM_PAGES][x+k] & (0x01<<(y+j)%8));
			}

		}

		x+=6;
		i++;
	}

    refresh_oled(matrix);
}

void draw_char_oled(char str, uint8_t matrix[NUM_PAGES][PAGE_SIZE], uint8_t x, uint8_t y)
{
    for(uint8_t k=0; k<6; k++)
    {
        for(uint8_t j=0; j<8; j++)
        {
            if((table_code[str-32][k]>>j & 0x01))
                matrix[(y+j)/NUM_PAGES][x+k] |= 0x01 << ((y+j)%8);
            else
                matrix[(y+j)/NUM_PAGES][x+k] = matrix[(y+j)/NUM_PAGES][x+k]^(matrix[(y+j)/NUM_PAGES][x+k] & (0x01<<(y+j)%8));
        }

    }

    refresh_oled(matrix);
}

void clear_oled(uint8_t matrix[NUM_PAGES][PAGE_SIZE])
{
    for(uint8_t i=0; i<NUM_PAGES; i++)
    {
        for(uint8_t j=0; j<PAGE_SIZE;j++)
          screen[i][j] = 0x00;
    }
    refresh_oled(screen);
}

uint8_t reverse(uint8_t b)
{
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void draw_bitmap_oled(uint8_t matrix[NUM_PAGES][PAGE_SIZE], const uint8_t *bitmap, uint16_t size, uint8_t weigth, uint8_t x, uint8_t y)
{
   
    uint8_t j=0;
    uint16_t k=0;
    while(k<size)
    {
        for(uint8_t i=0; i<=weigth; i++)
        {
            matrix[j][x+i]=reverse(bitmap[k]);
            k++;
        }
        j++;
    }

    refresh_oled(matrix);
    
}

void init_screen()
{
    if(i2c_check_response(I2C_DIR_OLED, 1000))
    {
        init_oled();
        clear_oled(screen);
        draw_bitmap_oled(screen, esimeico, 518, 64, 32, 0);
        debug("\n\n\n\n\n\n\nV %f\n", VERSION_APP);
        sleep_ms(2000);
    }

    else
    {
        while(1)
        {
            gpio_put(LED_STATUS, 1);
            sleep_ms(1000);
            gpio_put(LED_STATUS, 0);
            sleep_ms(1000);
        }
    }
 
}