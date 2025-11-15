using System;
using System.Runtime.InteropServices;

namespace OusterSdkCSharp;

public sealed class OusterLidarScan : IDisposable
{
    internal IntPtr Handle { get; private set; }
    internal XYZLut Lut { get; private set; }

    internal int Width { get; private set; }
    internal int Height { get; private set; }

    internal OusterLidarScan(IntPtr handle, XYZLut lut)
    {
        Handle = handle;
        Lut = lut;
        NativeMethods.ouster_lidar_scan_get_dimensions(Handle, out int w, out int h);
        Width = w;
        Height = h;
    }

    public void Dispose()
    {
        if (Handle != IntPtr.Zero)
        {
            NativeMethods.ouster_lidar_scan_destroy(Handle);
            Handle = IntPtr.Zero;
        }
        GC.SuppressFinalize(this);
    }

    public float[] GetXYZ(bool filterInvalid)
    {
        ulong maxPoints = (ulong)Width * (ulong)Height;
        var xyz = new float[maxPoints * 3];
        var ptr = Marshal.AllocHGlobal(sizeof(float) * (int)(maxPoints * 3));
        try
        {
            int rc = NativeMethods.ouster_lidar_scan_get_xyz(
                Handle, Lut.Handle, ptr, (UIntPtr)maxPoints, out var outPoints, filterInvalid ? 1 : 0);
            if (rc != 0) return Array.Empty<float>();
            int n = (int)outPoints;
            Marshal.Copy(ptr, xyz, 0, (int)(n * 3));
            if (n * 3 < xyz.Length)
            {
                Array.Resize(ref xyz, (int)(n * 3));
            }
            return xyz;
        }
        finally
        {
            Marshal.FreeHGlobal(ptr);
        }
    }

    public uint[] GetRange()
    {
        ulong count = (ulong)Width * (ulong)Height;
        var managed = new int[count];
        var ptr = Marshal.AllocHGlobal(sizeof(uint) * (int)count);
        try
        {
            int rc = NativeMethods.ouster_lidar_scan_get_field_u32(
                Handle, "RANGE", ptr, (UIntPtr)count, out var outCount);
            if (rc != 0) return Array.Empty<uint>();
            Marshal.Copy(ptr, managed, 0, (int)outCount);
            return Array.ConvertAll(managed, item => (uint)item);
        }
        finally { Marshal.FreeHGlobal(ptr); }
    }

    public ushort[] GetReflectivity(OusterLidarScan scan)
    {
        ulong count = (ulong)Width * (ulong)Height;
        var managed = new short[count];
        var ptr = Marshal.AllocHGlobal(sizeof(ushort) * (int)count);
        try
        {
            int rc = NativeMethods.ouster_lidar_scan_get_field_u16(scan.Handle, "REFLECTIVITY", ptr, (UIntPtr)count, out var outCount);
            if (rc != 0) return Array.Empty<ushort>();
            Marshal.Copy(ptr, managed, 0, (int)outCount);
            return Array.ConvertAll(managed, item => (ushort)item);
        }
        finally { Marshal.FreeHGlobal(ptr); }
    }
}

public sealed class XYZLut : IDisposable
{
    internal IntPtr Handle { get; private set; }

    internal XYZLut(IntPtr handle)
    {
        Handle = handle;
    }

    public void Dispose()
    {
        if (Handle != IntPtr.Zero)
        {
            NativeMethods.ouster_xyz_lut_destroy(Handle);
            Handle = IntPtr.Zero;
        }
        GC.SuppressFinalize(this);
    }
}

public sealed class OusterScanSource : IDisposable
{
    public IntPtr Handle { get; private set; }
    private int _width;
    private int _height;
    public int Width => _width;
    public int Height => _height;

    private XYZLut _lut = null!;

    public static OusterScanSource? Create(string hostname)
    {
        int rc = NativeMethods.ouster_scan_source_create(hostname, out var handle);
        if (rc != 0 || handle == IntPtr.Zero) return null;
        var src = new OusterScanSource(handle);
        src.RefreshDimensions();
        src._lut = src.CreateXYZLut();
        return src;
    }

    public XYZLut CreateXYZLut(bool useExtrinsics = true)
    {
        int flag = useExtrinsics ? 1 : 0;
        var lutPtr = NativeMethods.ouster_scan_source_create_xyz_lut(Handle, flag);
        return new XYZLut(lutPtr);
    }

    private OusterScanSource(IntPtr h) => Handle = h;

    private void RefreshDimensions()
    {
        if (Handle != IntPtr.Zero)
        {
            NativeMethods.ouster_scan_source_frame_dimensions(Handle, out _width, out _height);
        }
    }

    public string GetMetadata()
    {
        int len = NativeMethods.ouster_scan_source_get_metadata(Handle, IntPtr.Zero, UIntPtr.Zero);
        if (len <= 0) return string.Empty;
        var buf = Marshal.AllocHGlobal(len + 1);
        try
        {
            NativeMethods.ouster_scan_source_get_metadata(Handle, buf, (UIntPtr)(ulong)(len + 1));
            return Marshal.PtrToStringAnsi(buf) ?? string.Empty;
        }
        finally { Marshal.FreeHGlobal(buf); }
    }

    public OusterLidarScan? NextScan(int timeoutSec = 2)
    {
        var scanPtr = NativeMethods.ouster_scan_source_next_scan(Handle, timeoutSec);
        return scanPtr == IntPtr.Zero ? null : new OusterLidarScan(scanPtr, _lut);
    }

    public void Dispose()
    {
        if (Handle != IntPtr.Zero)
        {
            NativeMethods.ouster_scan_source_destroy(Handle);
            Handle = IntPtr.Zero;
        }
        GC.SuppressFinalize(this);
    }
}
