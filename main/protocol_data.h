#ifndef PROTOCOL_DATA_H_
#define PROTOCOL_DATA_H_
#include <stdbool.h>
#include <stdint.h>

/* [data data data] CRC32 SIZE16 SIGNATURE */

#define CMD_DATA_SIZE 10240
#define CMD_WIRE_SIZE (CMD_DATA_SIZE)

#define PD_SIGNATURE 0x50443031U
#define PD_SIGNATURE_STR "PD01"

typedef struct {
    uint32_t crc32;
    uint16_t size;
    uint8_t signature[4U];
} __attribute__((__packed__)) packet_header_t;

extern uint16_t wire_data_size;
extern uint8_t wire_data[CMD_WIRE_SIZE];
extern uint8_t fixed_data[CMD_DATA_SIZE];
void process_data(const uint8_t* data, uint16_t data_size);

#endif /* PROTOCOL_DATA_H_ */
