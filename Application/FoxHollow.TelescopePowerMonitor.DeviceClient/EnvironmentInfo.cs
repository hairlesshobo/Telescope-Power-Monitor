using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace FoxHollow.TelescopePowerMonitor.DeviceClient
{
    public class EnvironmentInfo : IDeviceLogType
    {
        public DateTimeOffset LastReadDtm { get; private set; } = default;
        public int LastReadUptime { get; private set; } = 0;

        public FloatValue Temperature { get; } = new FloatValue();
        public FloatValue Humidity { get; } = new FloatValue();

        internal bool ParseLogLine(string inputLine)
        {
            string[] parts = inputLine.Split('|');

            //    0             1         2           3                       4            
            // <ISO_DTM>|<UptimeSeconds>|ENV|<CurrentTemperatureC>|<CurrentHumidityPercent>

            if (parts.Length < 5)
                return false;

            if (!parts[2].Equals("ENV", StringComparison.InvariantCultureIgnoreCase))
                return false;

            this.LastReadDtm = DateTimeOffset.Parse(parts[0], null, DateTimeStyles.AssumeUniversal);
            this.LastReadUptime = Int32.Parse(parts[1]);

            this.Temperature.UpdateValue(parts[3], this.LastReadDtm);
            this.Humidity.UpdateValue(parts[4], this.LastReadDtm);

            this.Temperature.Valid = true;
            this.Humidity.Valid = true;

            return true;
        }

        internal void Reset()
        {
            this.LastReadDtm = default;
            this.LastReadUptime = 0;

            this.Temperature.Reset();
            this.Humidity.Reset();
        }
    }
}
