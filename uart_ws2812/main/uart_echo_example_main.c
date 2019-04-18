/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
/**
 * This is an example which echos any data it receives on UART1 back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: UART1
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below
 */

#define ECHO_TEST_TXD  (GPIO_NUM_4)
#define ECHO_TEST_RXD  (GPIO_NUM_5)
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)
#define WS2812BITS(n) (0x12 | ((~n & 0x01) << 6) |((~n & 0x02) << 2 )| ((~n & 0x04) >> 2 ))


const uint8_t bitTable[] = {WS2812BITS(0),WS2812BITS(1),WS2812BITS(2),WS2812BITS(3),
                            WS2812BITS(4),WS2812BITS(5),WS2812BITS(6),WS2812BITS(7),
                            WS2812BITS(8),WS2812BITS(9),WS2812BITS(10),WS2812BITS(11),
                            WS2812BITS(12),WS2812BITS(13),WS2812BITS(14),WS2812BITS(15)};

void computeRGBBits(uint8_t *buffer, uint8_t r, uint8_t g, uint8_t b){
    //GRB
    //G(7,5)
    buffer[0] = bitTable[(g & 0xe) >> 5];
    //G(4,2) 
    buffer[1] = bitTable[(g & 0x1c) >> 2];
    //G(1,0) | R(7)
    buffer[2] = bitTable[((g & 0x03) << 1) | (r >> 7) ];
    //R(6,4)
    buffer[3] = bitTable[(r & 0x70) >> 4];
    //R(3,1)
    buffer[4] = bitTable[(r & 0x0e) >> 1];
    //R(0) | B(7,6)
    buffer[5] = bitTable[(r & 0x01) | ((b & 0xc0) >> 6)];
    //B(5,3)
    buffer[6] = bitTable[(b & 0x38) >> 3];
    //B(2,0)
    buffer[7] = bitTable[(b & 0x07)];
}


static void echo_task()
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 2400000,
        .data_bits = UART_DATA_7_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_set_line_inverse(UART_NUM_1,UART_INVERSE_TXD);
    uart_driver_install(UART_NUM_1, 129, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(8);
    computeRGBBits(data,100,0,0);
    const TickType_t xDelay = 100 / portTICK_PERIOD_MS;
    uint8_t i = 0;

    while (1) {
        // Read data from the UART
        //int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        
        // Write data back to the UART
        computeRGBBits(data,(10*i)&0xff,0,0);
        uart_write_bytes(UART_NUM_1, (const char *) data, 8);
        vTaskDelay(xDelay);
        i++;
    }
}

void app_main()
{
    xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
}
