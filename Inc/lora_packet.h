#ifndef LORA_PACKET_H
#define LORA_PACKET_H

#include <stdbool.h>
#include <stdint.h>

#define START_SIGN_H              0x55
#define START_SIGN_L 			  0xAA
#define PACKT_TYPE_BEGINNING      0x01
#define PACKT_TYPE_DATA_PACKET    0x00

#define LIDAR_MAX_POINTS   		  251

typedef enum {
	PARSER_WAIT_HEADER_L,
	PARSER_WAIT_HEADER_H,
	PARSER_WAIT_CT,
	PARSER_WAIT_LSN,
	PARSER_WAIT_FSA_L,
	PARSER_WAIT_FSA_H,
	PARSER_WAIT_LSA_L,
	PARSER_WAIT_LSA_H,
	PARSER_WAIT_CS_L,
	PARSER_WAIT_CS_H,
	PARSER_WAIT_SAMPLE
} parser_state_t;

typedef struct
{
    uint16_t raw;

    float distance;

    float angle;

    float angle_correction;

} lidar_point_t;

typedef struct {

	lidar_point_t point[LIDAR_MAX_POINTS];
	uint16_t fsa;
	uint16_t lsa;
	uint16_t cs;
    uint8_t ct;
    uint8_t lsn;

} lidar_frame_t;

typedef struct {

	lidar_frame_t frame;
	uint8_t sample_buffer[2];
    uint8_t sample_index;
    uint8_t sample_byte_index;
    uint8_t temp_lsb;
    parser_state_t state;
    bool frame_ready;

} packet_parser_t;


void packet_parser_reset(packet_parser_t *p);
/*
uint16_t crc16_kermit(const uint8_t *data, uint16_t len);

bool crc16_is_valid(const packet_parser_t *p);
*/
void packet_parse_byte(packet_parser_t *p, uint8_t byte);

void packet_parse_frame(packet_parser_t *p, uint8_t *buffer, uint8_t len);

#endif // LORA_PACKET_H
