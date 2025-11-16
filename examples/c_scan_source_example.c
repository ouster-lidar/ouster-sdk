/*
 * Pure C example using scan source wrapper to collect a few scans and output
 * point clouds to CSV files.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ouster_c.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: c_scan_source_example <sensor_hostname>\n");
        return argc == 1 ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    const char* hostname = argv[1];

    ouster_scan_source_t* src = NULL;
    if (ouster_scan_source_create(hostname, &src) != 0 || !src) {
        fprintf(stderr, "Failed to create scan source\n");
        return EXIT_FAILURE;
    }

    int w = 0, h = 0;
    if (ouster_scan_source_frame_dimensions(src, &w, &h) != 0) {
        fprintf(stderr, "Failed to get frame dimensions\n");
        ouster_scan_source_destroy(src);
        return EXIT_FAILURE;
    }
    printf("Frame dimensions: %dx%d\n", w, h);

    size_t max_points = (size_t)w * (size_t)h;
    float* xyz = (float*)malloc(max_points * 3 * sizeof(float));
    if (!xyz) {
        fprintf(stderr, "Allocation failure\n");
        ouster_scan_source_destroy(src);
        return EXIT_FAILURE;
    }

    ouster_xyz_lut_t* lut =
        ouster_scan_source_create_xyz_lut(src, /*use_extrinsics=*/1);
    if (!lut) {
        fprintf(stderr, "Failed to create XYZ LUT\n");
        free(xyz);
        ouster_scan_source_destroy(src);
        return EXIT_FAILURE;
    }

    int scans_needed = 5;
    int scans_got = 0;
    while (scans_got < scans_needed) {
        ouster_lidar_scan_t* scan = ouster_scan_source_next_scan(src, 2);
        if (!scan) {
            printf("Timeout waiting for scan...\n");
            continue;
        }

        // Convert to XYZ
        size_t n_points = 0;
        if (ouster_lidar_scan_get_xyz(scan, lut, xyz, max_points, &n_points,
                                      /*filter_invalid=*/0) != 0) {
            fprintf(stderr, "Failed to convert scan to XYZ\n");
            ouster_lidar_scan_destroy(scan);
            break;
        }

        // Extract RANGE field as uint32 for optional inspection
        uint32_t* range = (uint32_t*)malloc(max_points * sizeof(uint32_t));
        size_t range_count = 0;
        if (range && ouster_lidar_scan_get_field_u32(
                         scan, "RANGE", 0, range, max_points, &range_count) == 0) {
            printf("First RANGE values: ");
            for (size_t i = 0; i < (range_count < 8 ? range_count : 8); ++i)
                printf("%u ", range[i]);
            printf("...\n");
        }
        free(range);

        char fname[64];
        snprintf(fname, sizeof(fname), "c_cloud_%d.csv", scans_got);
        FILE* f = fopen(fname, "w");
        if (!f) {
            fprintf(stderr, "Could not open output file %s\n", fname);
            ouster_lidar_scan_destroy(scan);
            break;
        }
        for (size_t i = 0; i < n_points; ++i) {
            fprintf(f, "%f,%f,%f\n", xyz[3 * i], xyz[3 * i + 1],
                    xyz[3 * i + 2]);
        }
        fclose(f);
        printf("Wrote %s with %zu points\n", fname, n_points);
        scans_got++;
        ouster_lidar_scan_destroy(scan);
    }

    free(xyz);
    ouster_xyz_lut_destroy(lut);
    ouster_scan_source_destroy(src);
    printf("Done.\n");
    return EXIT_SUCCESS;
}
