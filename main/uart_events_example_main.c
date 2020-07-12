#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "protocol_data.h"
#include "drv_canfdspi_api.h"
#include "drv_canfdspi_spi.h"
#include "drv_canfdspi_config.h"

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

static const char *TAG = "SYS";

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (2)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

#define BLOCK_SIZE 120

static void uart_event_task (void *pvParameters) {
    uart_event_t event;
    uint8_t data[BLOCK_SIZE];
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void* )&event, (portTickType)portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                {
                    size_t remaining_data = event.size;
                    while (remaining_data) {
                        size_t read_size = uart_read_bytes (EX_UART_NUM, data, MIN(remaining_data, sizeof(data)), portMAX_DELAY);
                        process_data(data,read_size);
                        remaining_data-=read_size;
                    }
                }
                    break;
                    //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    uart_flush_input (EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                    //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    uart_flush_input (EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                    //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                case UART_PATTERN_DET:
                    ESP_LOGE(TAG, "Unexpected UART_PATTERN_DET event");
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
//    uart_config_t uart_config =
//        { .baud_rate = 460800, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl =
//                UART_HW_FLOWCTRL_DISABLE };
//    uart_param_config (EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set (TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
//    uart_set_pin (EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver, and get the queue.
    uart_driver_install (EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);

    /* SPI Part */
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=100,
        .flags=SPICOMMON_BUSFLAG_MASTER
    };

    spi_device_interface_config_t devcfg={
        .clock_speed_hz=1*1000*1000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb = NULL,  //Specify pre-transfer callback to handle D/C line
    };

    //Initialize the SPI bus
    ret = spi_bus_initialize(CANFD_SPI, &buscfg, 0);
    ESP_ERROR_CHECK(ret);

    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(CANFD_SPI, &devcfg, &canfd_spi);
    ESP_ERROR_CHECK(ret);

    int8_t result;

    result=DRV_CANFDSPI_OperationModeSelect(0,CAN_CONFIGURATION_MODE);
    ESP_LOGI(TAG, "DRV_CANFDSPI_OperationModeSelect return %u", result);
    CAN_OPERATION_MODE mode=DRV_CANFDSPI_OperationModeGet(0);
    ESP_LOGI(TAG, "DRV_CANFDSPI_OperationModeGet return %u", mode);

    CAN_OSC_STATUS status;
    result=DRV_CANFDSPI_OscillatorStatusGet(0U, &status);
    ESP_LOGI(TAG, "DRV_CANFDSPI_OscillatorStatusGet return %u", result);
    ESP_LOGI(TAG, "PllReady=%u",status.PllReady);
    ESP_LOGI(TAG, "OscReady=%u",status.OscReady);
    ESP_LOGI(TAG, "SclkReady=%u",status.SclkReady);

    CAN_BUS_DIAGNOSTIC bd;
    result = DRV_CANFDSPI_BusDiagnosticsGet(0U, &bd);
    ESP_LOGI(TAG, "DRV_CANFDSPI_BusDiagnosticsGet return %u", result);
    ESP_LOGI(TAG, "errorFreeMsgCount = %u", bd.bF.errorFreeMsgCount);


    //Create a task to handler UART event from ISR
    xTaskCreate (uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

}
