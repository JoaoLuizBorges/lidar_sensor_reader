#include <lora_packet.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "lidar.h"

static polar_map_t map;

uint16_t lidar_checksum(packet_parser_t *p) {

    uint16_t cs = 0x55AA;

    cs ^= p->frame.fsa;

    for(uint8_t i=0; i<p->frame.lsn; i++)
    {
        cs ^= p->frame.point[i].raw;
    }

    cs ^= ((uint16_t)p->frame.lsn << 8) | p->frame.ct;

    cs ^= p->frame.lsa;

    return cs;
}

void lidar_process_frame(lidar_frame_t *frame) {

	float start_angle = (float)(frame->fsa>>1) / 64.0f;
	float end_angle = (float)(frame->lsa>>1) / 64.0f;

	float angle_diff = end_angle - start_angle;

	if(angle_diff < 0.0f) {

		angle_diff += 360.0f;
	}

	for(uint8_t i = 0; i < frame->lsn; i++) {

		lidar_point_t *point = &frame->point[i];
		point->distance = point->raw / 4.0f;

		if(point->distance > 0.0f) {
			point->angle_correction =
					atanf(21.8f *(155.3f - point->distance)/
							(155.3 * point->distance));
		} else {
			point->angle_correction = 0.0f;

		}

		point->angle = start_angle + (angle_diff * i) / frame->lsn +
						point->angle_correction;

		if(point->angle >= 360.0f) point->angle -= 360.0f;

		if(point->angle < 0.0f) point->angle += 360.0f;

	}
}

static void lidar_clear_map(void) {

	memset(map.valid, 0, sizeof(map.valid));

    for(int i = 0; i < LIDAR_MAP_SIZE; i++) {

        map.distance[i] = 0.0f;
    }
}

float lidar_get_distance(uint16_t angle)
{
    angle %= 360;

    if(map.valid[angle])
        return map.distance[angle];

    return -1.0f;
}

bool lidar_is_valid(uint16_t angle)
{
    return map.valid[angle % 360];
}

const polar_map_t *lidar_get_map(void)
{
    return &map;
}

void lidar_update_map(const lidar_frame_t *frame) {

	static float last_angle = 0.0f;

    for(uint8_t i = 0; i < frame->lsn; i++) {

        uint16_t angle = (uint16_t)frame->point[i].angle;

        if(angle < last_angle) {
        	lidar_clear_map();
        }

        last_angle = angle;

        float dist = frame->point[i].distance;

        if(dist <= 0.0f)
            continue;

        if(!map.valid[angle]) {
            map.valid[angle] = true;
            map.distance[angle] = dist;
        } else {

        	if(dist < map.distance[angle]) {
                map.distance[angle] = dist;
            }
        }
    }
}

void lidar_print_frame(const lidar_frame_t *frame)
{
    printf("\r\n========== FRAME ==========\r\n");

    printf("CT  : %02X\r\n", frame->ct);
    printf("LSN : %u\r\n", frame->lsn);
    printf("FSA : %04X (%.2f°)\r\n",
           frame->fsa,
           (float)(frame->fsa >> 1) / 64.0f);

    printf("LSA : %04X (%.2f°)\r\n",
           frame->lsa,
           (float)(frame->lsa >> 1) / 64.0f);

    printf("CS  : %04X\r\n", frame->cs);

    printf("---------------------------------------------\r\n");
    printf("Idx   Raw    Dist(mm)   Corr(°)   Angle(°)\r\n");
    printf("---------------------------------------------\r\n");

    for (uint8_t i = 0; i < frame->lsn; i++)
    {
        printf("%02u  %04X   %7.2f   %7.2f   %7.2f\r\n",
               i,
               frame->point[i].raw,
               frame->point[i].distance,
               frame->point[i].angle_correction,
               frame->point[i].angle);
    }

    printf("=============================================\r\n");
}
