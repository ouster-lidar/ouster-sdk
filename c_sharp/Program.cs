using System;
using System.Globalization;
using System.IO;
using System.Linq;
using OusterSdkCSharp;

internal static class Program
{
    private const int ScansNeeded = 5;

    public static void Main(string[] args)
    {
        if (args.Length < 1)
        {
            Console.Error.WriteLine("Usage: dotnet run --project c_sharp <sensor_hostname>");
            return;
        }
        var hostname = args[0];

        using var source = OusterScanSource.Create(hostname);
        if (source is null)
        {
            Console.Error.WriteLine("Failed to create scan source");
            return;
        }

        Console.WriteLine($"Frame dimensions: {source.Width}x{source.Height}");

        int scansGot = 0;
        while (scansGot < ScansNeeded)
        {
            using var scan = source.NextScan(2);
            if (scan is null)
            {
                Console.WriteLine("Timeout waiting for scan...");
                continue;
            }
            var xyz = scan.GetXYZ(filterInvalid: true);
            var range = scan.GetRange();
            Console.WriteLine(
                $"Scan {scansGot}: points={xyz.Length / 3}, rangeSample=[{string.Join(' ', range.Take(8))} ...]");

            var fname = $"cs_cloud_{scansGot}.csv";
            using var sw = new StreamWriter(fname);
            sw.WriteLine("x,y,z");
            for (int i = 0; i < xyz.Length; i += 3)
            {
                sw.WriteLine(string.Create(CultureInfo.InvariantCulture, $"{xyz[i]},{xyz[i+1]},{xyz[i+2]}"));
            }
            Console.WriteLine($"Wrote {fname}");
            ++scansGot;
        }
        Console.WriteLine("Done.");
    }
}
