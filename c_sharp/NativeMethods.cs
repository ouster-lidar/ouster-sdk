using System;
using System.Runtime.InteropServices;

namespace OusterSdkCSharp;

internal static class NativeConstants
{
    public const int OU_CLIENT_TIMEOUT = 0;
    public const int OU_CLIENT_ERROR = 1;
    public const int OU_CLIENT_LIDAR_DATA = 2;
    public const int OU_CLIENT_IMU_DATA = 4;
    public const int OU_CLIENT_EXIT = 8;
}

internal static class NativeMethods
{
    private const string LinuxLib = "ouster_c"; // resolves to libouster_c.so
    private const string MacLib = "ouster_c";   // resolves to libouster_c.dylib
    private const string WindowsLib = "ouster_c"; // ouster_c.dll

#if WINDOWS
    private const string Lib = WindowsLib;
#elif OSX
    private const string Lib = MacLib;
#else
    private const string Lib = LinuxLib;
#endif

    // Client API
    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern IntPtr ouster_client_create(string hostname, int lidar_port, int imu_port);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void ouster_client_destroy(IntPtr client);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ouster_client_poll(IntPtr client, int timeout_sec);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ouster_client_get_metadata(IntPtr client, IntPtr buffer, UIntPtr capacity);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ouster_client_fetch_and_parse_metadata(IntPtr client, int timeout_sec);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ouster_client_get_frame_dimensions(IntPtr client, out int width, out int height);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ouster_client_get_packet_sizes(IntPtr client, out UIntPtr lidar_packet_size, out UIntPtr imu_packet_size);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ouster_client_read_lidar_packet(IntPtr client, IntPtr buf, UIntPtr buf_size);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ouster_client_read_imu_packet(IntPtr client, IntPtr buf, UIntPtr buf_size);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ouster_client_get_lidar_port(IntPtr client);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ouster_client_get_imu_port(IntPtr client);

    // Scan Source API
    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ouster_scan_source_create(string hostname, out IntPtr out_source);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void ouster_scan_source_destroy(IntPtr source);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ouster_scan_source_frame_dimensions(IntPtr source, out int width, out int height);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ouster_scan_source_get_metadata(IntPtr source, IntPtr buffer, UIntPtr capacity);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern IntPtr ouster_scan_source_next_scan(IntPtr source, int timeout_sec);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void ouster_lidar_scan_destroy(IntPtr scan);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void ouster_lidar_scan_get_dimensions(IntPtr scan, out int width, out int height);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ouster_lidar_scan_get_field_u32(IntPtr scan, string field_name, IntPtr out_buf, UIntPtr capacity, out UIntPtr out_count);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ouster_lidar_scan_get_field_u16(IntPtr scan, string field_name, IntPtr out_buf, UIntPtr capacity, out UIntPtr out_count);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern int ouster_lidar_scan_get_xyz(IntPtr scan, IntPtr lut, IntPtr xyz_out, UIntPtr capacity_points, out UIntPtr out_points, int filter_invalid);

    // XYZ LUT API
    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern IntPtr ouster_scan_source_create_xyz_lut(IntPtr source, int use_extrinsics);

    [DllImport(Lib, CallingConvention = CallingConvention.Cdecl)]
    internal static extern void ouster_xyz_lut_destroy(IntPtr lut);
}
