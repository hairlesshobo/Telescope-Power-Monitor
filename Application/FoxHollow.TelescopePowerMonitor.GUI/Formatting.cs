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
            => $"{input:0.0}A";

        internal static string FormatPercentage(float input)
            => $"{input:0}%";

        internal static string FormatUptime(int seconds)
        {
            List<string> returnParts = new List<string>();

            if (seconds > 86400)
            {
                int days = seconds / 86400;
                returnParts.Add(days.ToString() + "d");
                seconds -= days * 86400;
            }

            if (seconds > 3600)
            {
                int hours = seconds / 3600;
                returnParts.Add(hours.ToString() + "h");
                seconds -= hours * 3600;
            }

            if (seconds > 60)
            {
                int minutes = seconds / 60;
                returnParts.Add(minutes.ToString() + "m");
                seconds -= minutes * 60;
            }

            if (seconds > 0)
            {
                returnParts.Add(seconds.ToString() + "s");
            }

            return String.Join(" ", returnParts);
        }
    }
}
