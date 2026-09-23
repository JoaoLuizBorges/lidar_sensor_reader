#ifndef LIDAR_H
#define LIDAR_H

#include <lora_packet.h>
#include <stdio.h>
#include <stdbool.h>

#define LIDAR_MAP_SIZE 360

typedef struct
{
    float distance[LIDAR_MAP_SIZE];
    bool valid[LIDAR_MAP_SIZE];

} polar_map_t;

uint16_t lidar_checksum(packet_parser_t *p);

static void lidar_clear_map(void);

float lidar_get_distance(uint16_t angle);

bool lidar_is_valid(uint16_t angle);

const polar_map_t *lidar_get_map(void);

void lidar_update_map(const lidar_frame_t *frame);

void lidar_process_frame(lidar_frame_t *frame);

void lidar_print_frame(const lidar_frame_t *frame);


#endif //LIDAR_H
