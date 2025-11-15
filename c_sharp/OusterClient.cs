using System;
using System.Runtime.InteropServices;

namespace OusterSdkCSharp;

public sealed class OusterClient : IDisposable
{
    public IntPtr Handle { get; private set; }
    public bool IsValid => Handle != IntPtr.Zero;

    public static OusterClient? Create(string hostname, int lidarPort = 0, int imuPort = 0)
    {
        var h = NativeMethods.ouster_client_create(hostname, lidarPort, imuPort);
        return h == IntPtr.Zero ? null : new OusterClient(h);
    }

    private OusterClient(IntPtr handle) => Handle = handle;

    public int FetchAndParseMetadata(int timeoutSec = 30) => NativeMethods.ouster_client_fetch_and_parse_metadata(Handle, timeoutSec);

    public string GetMetadata()
    {
        int len = NativeMethods.ouster_client_get_metadata(Handle, IntPtr.Zero, UIntPtr.Zero);
        if (len <= 0) return string.Empty;
        var buf = Marshal.AllocHGlobal(len + 1);
        try
        {
            NativeMethods.ouster_client_get_metadata(Handle, buf, (UIntPtr)(ulong)(len + 1));
            return Marshal.PtrToStringAnsi(buf) ?? string.Empty;
        }
        finally { Marshal.FreeHGlobal(buf); }
    }

    public (int Width, int Height) GetFrameDimensions()
    {
        NativeMethods.ouster_client_get_frame_dimensions(Handle, out int w, out int h);
        return (w, h);
    }

    public (ulong LidarPacketSize, ulong ImuPacketSize) GetPacketSizes()
    {
        NativeMethods.ouster_client_get_packet_sizes(Handle, out UIntPtr lidar, out UIntPtr imu);
        return ((ulong)lidar, (ulong)imu);
    }

    public int Poll(int timeoutSec = 1) => NativeMethods.ouster_client_poll(Handle, timeoutSec);

    public byte[]? ReadLidarPacket()
    {
        var sizes = GetPacketSizes();
        if (sizes.LidarPacketSize == 0) return null;
        var arr = new byte[sizes.LidarPacketSize];
        var ptr = Marshal.AllocHGlobal(arr.Length);
        try
        {
            int ok = NativeMethods.ouster_client_read_lidar_packet(Handle, ptr, (UIntPtr)(ulong)arr.Length);
            if (ok == 1) Marshal.Copy(ptr, arr, 0, arr.Length);
            return ok == 1 ? arr : null;
        }
        finally { Marshal.FreeHGlobal(ptr); }
    }

    public void Dispose()
    {
        if (Handle != IntPtr.Zero)
        {
            NativeMethods.ouster_client_destroy(Handle);
            Handle = IntPtr.Zero;
        }
        GC.SuppressFinalize(this);
    }
}
