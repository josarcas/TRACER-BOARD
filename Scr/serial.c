/*INCLUDES********************************************************************************************/
#include "string.h"

#include "serial.h"

/*PROTOTYPES*****************************************************************************************/
static void serial_receive_task(void *arg); 

/*FUNCTIONS*******************************************************************************************/
static void serial_receive_task(void *arg)
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

void serial_init()
{
    uart_init(UART_PORT, UART_BAUDRATE);
    gpio_set_function(UART_PIN_RX, GPIO_FUNC_UART);
    gpio_set_function(UART_PIN_TX, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_PORT, false, false);
    uart_set_format(UART_PORT, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_PORT, true);
}

void serial_transmit(char *str)
{

}