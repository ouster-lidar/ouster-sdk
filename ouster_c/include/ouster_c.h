/**
 * Minimal C wrapper for a subset of the Ouster SDK sensor client API.
 *
 * This header exposes an opaque handle and a small collection of functions
 * sufficient to connect to a sensor, obtain metadata, derive packet sizes,
 * poll for data readiness, and read raw lidar/imu packets.
 *
 * The wrapper purposely limits scope; for advanced batching and point cloud
 * generation use the native C++ API.
 */
#ifndef OUSTER_C_WRAPPER_H
#define OUSTER_C_WRAPPER_H

#include <stddef.h>
#include <stdint.h>

/* Export macro for building shared library with proper symbol visibility */
#if defined(_WIN32) || defined(__CYGWIN__)
    #ifdef OUSTER_C_BUILD
        #define OUSTER_C_API __declspec(dllexport)
    #else
        #define OUSTER_C_API __declspec(dllimport)
    #endif
#else
    #if __GNUC__ >= 4
        #define OUSTER_C_API __attribute__((visibility("default")))
    #else
        #define OUSTER_C_API
    #endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque client handle */
typedef struct ouster_client ouster_client_t;

/* Mirror of client_state enum */
typedef enum {
    OU_CLIENT_TIMEOUT = 0,
    OU_CLIENT_ERROR = 1,
    OU_CLIENT_LIDAR_DATA = 2,
    OU_CLIENT_IMU_DATA = 4,
    OU_CLIENT_EXIT = 8
} ouster_client_state_t;

/* Create a client (short-form init that does not push configuration). */
OUSTER_C_API ouster_client_t* ouster_client_create(const char* hostname, int lidar_port,
                                                   int imu_port);

/* Destroy and free all resources. */
OUSTER_C_API void ouster_client_destroy(ouster_client_t* client);

/* Poll for up to timeout_sec seconds; returns bitmask of ouster_client_state_t.
 */
OUSTER_C_API int ouster_client_poll(ouster_client_t* client, int timeout_sec);

/* Fetch metadata from the sensor. Returns length copied into buffer (truncates
 * if capacity insufficient). */
OUSTER_C_API int ouster_client_get_metadata(ouster_client_t* client, char* buffer,
                                            size_t capacity);

/* Fetch and parse metadata, caching sensor_info / packet_format internally.
 * Returns 0 on success, non-zero on failure. */
OUSTER_C_API int ouster_client_fetch_and_parse_metadata(ouster_client_t* client,
                                                        int timeout_sec);

/* After parsing metadata, query frame dimensions. Returns 0 on success. */
OUSTER_C_API int ouster_client_get_frame_dimensions(const ouster_client_t* client,
                                                    int* width, int* height);

/* After parsing metadata, query packet sizes. Returns 0 on success. */
OUSTER_C_API int ouster_client_get_packet_sizes(const ouster_client_t* client,
                                                size_t* lidar_packet_size,
                                                size_t* imu_packet_size);

/* Read a lidar packet into user-provided buffer (must be >= lidar_packet_size).
 * Returns 1 if packet read, 0 otherwise. */
OUSTER_C_API int ouster_client_read_lidar_packet(ouster_client_t* client, uint8_t* buf,
                                                 size_t buf_size);

/* Read an imu packet into user-provided buffer (must be >= imu_packet_size).
 * Returns 1 if packet read, 0 otherwise. */
OUSTER_C_API int ouster_client_read_imu_packet(ouster_client_t* client, uint8_t* buf,
                                               size_t buf_size);

/* Access chosen UDP ports. */
OUSTER_C_API int ouster_client_get_lidar_port(const ouster_client_t* client);
OUSTER_C_API int ouster_client_get_imu_port(const ouster_client_t* client);

/* ================= Scan Source High-Level API ================= */

typedef struct ouster_scan_source ouster_scan_source_t;
typedef struct ouster_lidar_scan ouster_lidar_scan_t;
typedef struct ouster_xyz_lut ouster_xyz_lut_t;

/* Create a scan source for a single sensor hostname (auto UDP dest). Returns 0
 * on success. */
OUSTER_C_API int ouster_scan_source_create(const char* hostname,
                                           ouster_scan_source_t** out_source);

/* Destroy scan source */
OUSTER_C_API void ouster_scan_source_destroy(ouster_scan_source_t* source);

/* Get frame dimensions (width = columns_per_frame, height = pixels_per_column).
 */
OUSTER_C_API int ouster_scan_source_frame_dimensions(const ouster_scan_source_t* source,
                                                     int* width, int* height);

/* Get metadata JSON snippet (like client). Returns bytes copied (or full length
 * if buffer null). */
OUSTER_C_API int ouster_scan_source_get_metadata(ouster_scan_source_t* source, char* buffer,
                                                 size_t capacity);

/* Blocking fetch of next scan up to timeout_sec seconds.
 * Returns pointer to newly allocated scan handle on success.
 * Returns NULL on timeout or error.
 */
OUSTER_C_API ouster_lidar_scan_t* ouster_scan_source_next_scan(ouster_scan_source_t* source,
                                                               int timeout_sec);

/* Destroy a scan handle returned by ouster_scan_source_next_scan */
OUSTER_C_API void ouster_lidar_scan_destroy(ouster_lidar_scan_t* scan);

/* Get scan dimensions (width = columns_per_frame, height = pixels_per_column).
 */
OUSTER_C_API void ouster_lidar_scan_get_dimensions(const ouster_lidar_scan_t* scan,
                                                   int* width, int* height);

/* Extract a channel field as uint32_t array.
 * out_buf capacity (in elements) must be >= number of pixels (w*h).
 * Writes count to out_count. Returns 0 on success, -1 unknown field, -2
 * capacity insufficient, -3 invalid args.
 */
OUSTER_C_API int ouster_lidar_scan_get_field_u32(const ouster_lidar_scan_t* scan,
                                                 const char* field_name, int destagger,
                                                 uint32_t* out_buf, size_t capacity,
                                                 size_t* out_count);

/* Extract a channel field as uint16_t array.
 * Same return conventions as above.
 */
OUSTER_C_API int ouster_lidar_scan_get_field_u16(const ouster_lidar_scan_t* scan,
                                                 const char* field_name, int destagger,
                                                 uint16_t* out_buf, size_t capacity,
                                                 size_t* out_count);

/* Extract a channel field as uint8_t array.
 * Same return conventions as above.
 */
OUSTER_C_API int ouster_lidar_scan_get_field_u8(const ouster_lidar_scan_t* scan,
                                                const char* field_name, int destagger,
                                                uint8_t* out_buf, size_t capacity,
                                                size_t* out_count);

/* Convert scan to XYZ using scan source's lookup table.
 * xyz_out: float array length >= capacity_points * 3.
 * If filter_invalid != 0, skips points with all-zero coordinates.
 * Writes number of points stored to out_points.
 * Returns 0 on success, -2 insufficient capacity (when not filtering), -3 bad
 * args.
 */
OUSTER_C_API int ouster_lidar_scan_get_xyz(const ouster_lidar_scan_t* scan,
                                           const ouster_xyz_lut_t* xyz_lut, float* xyz_out,
                                           size_t capacity_points, size_t* out_points,
                                           int filter_invalid);

/* ================= Explicit XYZLut Management ================= */

/* Create an XYZ lookup table from a scan source. use_extrinsics != 0 to
 * include extrinsics in the LUT calculation. Returns NULL on error. */
OUSTER_C_API ouster_xyz_lut_t* ouster_scan_source_create_xyz_lut(const ouster_scan_source_t* source,
                                                                 int use_extrinsics);

/* Destroy an XYZLut handle */
OUSTER_C_API void ouster_xyz_lut_destroy(ouster_xyz_lut_t* lut);


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* OUSTER_C_WRAPPER_H */
