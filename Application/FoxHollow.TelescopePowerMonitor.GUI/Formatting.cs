using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FoxHollow.TelescopePowerMonitor.GUI
{
    internal static class Formatting
    {
        internal static string Empty => "----";
        internal static string EmptyTemperature => "--- °C";
        internal static string EmptyVoltage => "---- v";
        internal static string EmptyAmperage => "---- A";
        

        internal static string FormatTemperature(float input)
            => $"{input:0.0}°C";

        internal static string FormatVoltage(float input)
            => $"{input:00.0}v";

        internal static string FormatAmperage(float input)
            => $"{input:0.00}A";

        internal static string FormatPercentage(float input)
            => $"{input:0}%";
    }
}
