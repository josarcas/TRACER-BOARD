#ifndef INC_SERIAL_H
#define INC_SERIAL_H
/*INCLUDES********************************************************************************************/
#include "hardware/uart.h"

/*DEFINES*********************************************************************************************/
#ifndef UART_BUS
#define UART_BUS
#define UART_PORT                   uart0
#define UART_BAUDRATE               115200
#define UART_PIN_RX                 1
#define UART_PIN_TX                 0
#endif

#define MAX_SIZE_BUFFER_RX          50
#define MAX_SIZE_BUFFER_TX          25
#define UART_MAX_TIMEOUT            10

/*TYPEDEFS******************************************************************************************/
typedef struct{
    char cmd;
#ifndef MAX_SIZE_INSTRUCTION_ARG
#define MAX_SIZE_INSTRUCTION_ARG    40
#endif
    char arg[MAX_SIZE_INSTRUCTION_ARG];

}instruction_t;

/*PROTOTYPES****************************************************************************************/
void serial_init();
void serial_transmit(char *str);

#endif