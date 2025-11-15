/*
 * Minimal pure-C example using the C wrapper library (ouster_c).
 * Connects to a sensor, fetches metadata, prints basic info, then polls
 * and reads a handful of lidar packets.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ouster_c.h"

int main(int argc, char** argv) {
    if (argc < 4) {
        fprintf(stderr,
                "Usage: c_client_example <sensor_hostname> <lidar_port> "
                "<imu_port>\n");
        return argc == 1 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    const char* hostname = argv[1];
    int lidar_port = atoi(argv[2]);
    int imu_port = atoi(argv[3]);
    ouster_client_t* cli = ouster_client_create(hostname, lidar_port, imu_port);
    if (!cli) {
        fprintf(stderr, "Failed to create Ouster client for %s\n", hostname);
        return EXIT_FAILURE;
    }

    if (ouster_client_fetch_and_parse_metadata(cli, 30) != 0) {
        fprintf(stderr, "Failed to fetch/parse metadata\n");
        ouster_client_destroy(cli);
        return EXIT_FAILURE;
    }

    int w = 0, h = 0;
    size_t lidar_sz = 0, imu_sz = 0;
    ouster_client_get_frame_dimensions(cli, &w, &h);
    ouster_client_get_packet_sizes(cli, &lidar_sz, &imu_sz);
    printf(
        "Connected: frame dimensions %dx%d, lidar packet size %zu, imu packet "
        "size %zu\n",
        w, h, lidar_sz, imu_sz);

    /* Print a small snippet of metadata */
    char meta_buf[512];
    int meta_copied =
        ouster_client_get_metadata(cli, meta_buf, sizeof(meta_buf));
    printf("Metadata snippet (%d bytes copied):\n%.*s\n", meta_copied,
           meta_copied, meta_buf);

    /* Poll and read up to 5 lidar packets */
    uint8_t* lidar_buf = (uint8_t*)malloc(lidar_sz);
    if (!lidar_buf) {
        fprintf(stderr, "Allocation failed\n");
        ouster_client_destroy(cli);
        return EXIT_FAILURE;
    }

    int received = 0;
    while (received < 5) {
        int st = ouster_client_poll(cli, 1);
        if (st & OU_CLIENT_LIDAR_DATA) {
            if (ouster_client_read_lidar_packet(cli, lidar_buf, lidar_sz)) {
                printf("Got lidar packet %d/%d (first byte: %u)\n",
                       received + 1, 5, (unsigned)lidar_buf[0]);
                received++;
            }
        } else if (st & OU_CLIENT_ERROR) {
            fprintf(stderr, "Client error during poll\n");
            break;
        } else if (st & OU_CLIENT_TIMEOUT) {
            printf("Poll timeout; retrying...\n");
        }
    }

    free(lidar_buf);
    ouster_client_destroy(cli);
    printf("Done.\n");
    return EXIT_SUCCESS;
}
