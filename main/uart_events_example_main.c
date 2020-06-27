/* UART Events Example

 This example code is in the Public Domain (or CC0 licensed, at your option.)

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "uart_events";

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (2)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
#define CMD_COMPRESSED_SIZE 10240
static QueueHandle_t uart0_queue;

static void uart_event_task (void *pvParameters) {
    uart_event_t event;
    size_t buffered_size, data_size = 0;
    bool invalid_buffer = false;
    static uint8_t dtmp [CMD_COMPRESSED_SIZE];
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void* )&event, (portTickType)portMAX_DELAY)) {
//            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch (event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                 other types of events. If we take too much time on data event, the queue might
                 be full.*/
                case UART_DATA:
//                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    if ((event.size + data_size) > CMD_COMPRESSED_SIZE) {
                        data_size += event.size;
                        invalid_buffer = true;
                    } else {
                        size_t read_size = uart_read_bytes (EX_UART_NUM, dtmp + data_size, event.size, portMAX_DELAY);
                        data_size += read_size;
                    }
                    break;
                    //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input (EX_UART_NUM);
                    invalid_buffer = true;
                    xQueueReset(uart0_queue);
                    break;
                    //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input (EX_UART_NUM);
                    invalid_buffer = true;
                    xQueueReset(uart0_queue);
                    break;
                    //Event of UART RX break detected
                case UART_BREAK:
                    invalid_buffer = true;
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                    //Event of UART parity check error
                case UART_PARITY_ERR:
                    invalid_buffer = true;
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                    //Event of UART frame error
                case UART_FRAME_ERR:
                    invalid_buffer = true;
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                    //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len (EX_UART_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos (EX_UART_NUM);
  //                  ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input (EX_UART_NUM);
                    } else {
                        if ((pos + data_size) > CMD_COMPRESSED_SIZE) {
                            uart_read_bytes (EX_UART_NUM, dtmp, pos, portMAX_DELAY); //Skip data
                            data_size += pos;
                            invalid_buffer = true;
                        } else {
                            size_t read_size = uart_read_bytes (EX_UART_NUM, dtmp + data_size, pos, portMAX_DELAY);
                            data_size += read_size;
                        }
                        uint8_t pat [PATTERN_CHR_NUM + 1];
                        uart_read_bytes (EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                    }
                    if (invalid_buffer) {
                        ESP_LOGI(TAG, "[INVALID_BUFFER] size: %u", data_size);
                    } else {
                        ESP_LOGI(TAG, "[COMMAND] size: %u", data_size);
                    }
                    data_size = 0;
                    invalid_buffer = false;
                    break;
                    //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    vTaskDelete (NULL);
}

void app_main () {
    esp_log_level_set (TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config =
        { .baud_rate = 2000000, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl =
                UART_HW_FLOWCTRL_DISABLE };
    uart_param_config (EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set (TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin (EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver, and get the queue.
    uart_driver_install (EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);

    //Set uart pattern detect function.
    uart_enable_pattern_det_intr (EX_UART_NUM, '+', PATTERN_CHR_NUM, 10000, 10, 10);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset (EX_UART_NUM, 20);

    //Create a task to handler UART event from ISR
    xTaskCreate (uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}
