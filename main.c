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

#include "Inc/screen.h"
#include "Inc/global_variables.h"

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
void gui_task(void *arg);

/*---------------------------------------------MAIN FUNCTIION----------------------------------------------------*/
int main()
{
    //SYSTEM INIT-------------------------------------------------------------------------------------------------
    stdio_init_all();
    set_sys_clock_khz(130000, true);

    init_digital_outputs();

    init_serial();
    init_i2c_bus();
    //init_screen();
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
    //xTaskCreate(&system_status_task, "system status", 1024, NULL, 1, NULL);
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
    return;
}

/*
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
*/

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
/*
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
*/
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

void gui_task(void *arg)
{
    oled_init();
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