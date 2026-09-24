#include <lora_packet.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "lidar.h"

extern uint8_t psk[32];

extern uint8_t seq;

void packet_parser_reset(packet_parser_t *p) {
	
	p->state = PARSER_WAIT_HEADER_L;
	memset(&p->frame, 0, sizeof(p->frame));
	p->frame_ready = false;
	
}
/*
uint16_t crc16_kermit(const uint8_t *data, uint16_t len) {
	
    uint16_t crc = 0x0000;

    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }

    return crc;
}

bool crc16_is_valid(const packet_parser_t *p) {
	
    uint16_t crc_calc;
	static uint8_t rx[3 + MAX_PAYLOAD];
	size_t crc_len = 0;
	
 	rx[crc_len++] = START_BYTE;
    rx[crc_len++] = p->len;
    	
	memcpy(&rx[crc_len], p->payload, p->len);
    crc_len += p->len;
    crc_calc = crc16_kermit(rx, crc_len);
    
    return (crc_calc == p->crc_rx);
}
*/

void packet_parse_byte(packet_parser_t *p, uint8_t byte) { 
    
    switch (p->state) {

	    case PARSER_WAIT_HEADER_L:
//	    	printf("PARSER_WAIT_HEADER_L\r\n");
	        if (byte == START_SIGN_L) {
	            p->state = PARSER_WAIT_HEADER_H;
	        } else {
	        	p->state = PARSER_WAIT_HEADER_L;
	        }

	        break;
	        	
	    case PARSER_WAIT_HEADER_H:
//	    	printf("PARSER_WAIT_HEADER_H\r\n");
	        if (byte == START_SIGN_H) {
	            p->state = PARSER_WAIT_CT;
	        } else {
	        	p->state = PARSER_WAIT_HEADER_L;
	        }

	        break;

	    case PARSER_WAIT_CT:
//	    	printf("PARSER_WAIT_CT\r\n");
	    	if (byte == PACKT_TYPE_BEGINNING || byte == PACKT_TYPE_DATA_PACKET) {
	    		p->state = PARSER_WAIT_LSN;
	    		p->frame.ct = byte;
	    	} else {
	    		p->state = PARSER_WAIT_HEADER_L;
	    	}

	    	break;

	    case PARSER_WAIT_LSN:
//	    	printf("PARSER_WAIT_LSN\r\n");
	    	p->frame.lsn = byte;
	    	p->sample_index = 0;
	    	p->sample_byte_index = 0;
	    	p->state = PARSER_WAIT_FSA_L;

	    	break;

	    case PARSER_WAIT_FSA_L:
//	    	printf("PARSER_WAIT_FSA_L\r\n");
	    	p->temp_lsb = byte;
	    	p->state = PARSER_WAIT_FSA_H;

	    	break;

	    case PARSER_WAIT_FSA_H:
//	    	printf("PARSER_WAIT_FSA_H\r\n");
	    	p->frame.fsa = ((uint16_t)byte << 8) | p->temp_lsb;
	    	p->state = PARSER_WAIT_LSA_L;

	    	break;

	    case PARSER_WAIT_LSA_L:
//	    	printf("PARSER_WAIT_LSA_L\r\n");
	    	p->temp_lsb = byte;
	    	p->state = PARSER_WAIT_LSA_H;

	    	break;

	    case PARSER_WAIT_LSA_H:
//	    	printf("PARSER_WAIT_LSA_H\r\n");
	    	p->frame.lsa = ((uint16_t)byte << 8) | p->temp_lsb;
			p->state = PARSER_WAIT_CS_L;

			break;

	    case PARSER_WAIT_CS_L:
//	    	printf("PARSER_WAIT_CS_L\r\n");
	    	p->temp_lsb = byte;
	    	p->state = PARSER_WAIT_CS_H;

	    	break;

	    case PARSER_WAIT_CS_H:
//	    	printf("PARSER_WAIT_CS_H\r\n");
	    	p->frame.cs = ((uint16_t)byte << 8) | p->temp_lsb;
	    	p->state = PARSER_WAIT_SAMPLE;

	    	break;

	    case PARSER_WAIT_SAMPLE:
//	    	printf("PARSER_WAIT_SAMPLE\r\n");
	    	p->sample_buffer[p->sample_byte_index++] = byte;

	    	    if (p->sample_byte_index == 2) {

	    	        p->frame.point[p->sample_index].raw =
	    	            ((uint16_t)p->sample_buffer[1] << 8) |
	    	             p->sample_buffer[0];

	    	        p->sample_byte_index = 0;

	    	        p->sample_index++;

	    	        if (p->sample_index == p->frame.lsn) {

						p->frame_ready = true;

	    	        	if (lidar_checksum(p) == p->frame.cs) {
							p->frame_ready = true;
						} else {
							packet_parser_reset(p);
						}
	    	        	return;
	    	        }
	    	    }

	    	break;

		}
}

void packet_parse_frame(packet_parser_t *p, uint8_t *buffer, uint8_t len) {
	
	if (!p || !buffer) return;
	
    for (uint8_t i = 0; i < len; i++)
    {
        packet_parse_byte(p, buffer[i]);

        if (p->frame_ready)
        {
        	break;
        }
    }
}
