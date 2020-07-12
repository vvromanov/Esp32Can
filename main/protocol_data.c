#include <string.h>
#include "protocol_data.h"
#include "esp_log.h"
#include "sys\param.h"

uint16_t wire_data_size = 0;
uint8_t wire_data [CMD_WIRE_SIZE];
uint8_t fixed_data [CMD_DATA_SIZE];

const uint8_t signature [] =
    { 0x50U, 0x44U, 0x30U, 0x31U };

static const char *TAG = "PD";

typedef enum {
    pd_no_data,
} pd_state_t;

static pd_state_t pd_state;

void process_data (const uint8_t *data, uint16_t data_size) {
    ESP_LOGI(TAG, "Get %u bytes now buffer has %u", data_size, wire_data_size);
//    memcpy (wire_data + wire_data_size, data, data_size);
//    size_t start_index = 0;
//    if (wire_data_size>3) {
//        start_index = wire_data_size - 3;
//    }
//    wire_data_size += data_size;
//    uint8_t* sig_ptr = (uint8_t*)memmem(wire_data + start_index, wire_data_size-start_index, signature, sizeof(signature));
//    switch (pd_state) {
//        case pd_no_data:
//            if (wire_data_size >= sizeof(packet_header_t)) {
//                uint8_t* sig_ptr = (uint8_t*)memmem(wire_data, wire_data_size, signature, sizeof(signature));
//                if (sig_ptr) {
//                    uint16_t sig_pos = sig_ptr - wire_data;
//                    ESP_LOGI(TAG, "Signature found at %u", sig_pos);
//                } else {
//                    memmove(wire_data, wire_data + wire_data_size - 3, 3);
//                    wire_data_size = 3;
//                }
//            }
//            break;
//    }
}
