using System;
using System.Collections.Generic;
using System.Text;

namespace FoxHollow.TelescopePowerMonitor.DeviceClient
{
    public class PowerInfo : IDeviceLogType
    {
        public DateTimeOffset LastReadDtm { get; private set; } = default;
        public int LastReadUptime { get; private set; } = 0;

        public FloatValue Voltage { get; } = new FloatValue();
        public FloatValue BatteryAmperage{ get; } = new FloatValue();
        public FloatValue LoadAmperage { get; } = new FloatValue();
        public FloatValue SolarAmperage { get; } = new FloatValue();
        public FloatValue AcAmperage { get; } = new FloatValue();

        public FloatValue BatterySoc { get; } = new FloatValue();
        public FloatValue BatteryCapacityAh { get; } = new FloatValue();

        public bool ParseSuccess { get; set; } = false;

        internal void ParseLogLine(string inputLine)
        {
            string[] parts = inputLine.Split('|');

            //    0             1         2      3           4            5          6         7            8                 9
            // <ISO_DTM>|<UptimeSeconds>|PWR|<Voltage>|<BatteryAmps>|<LoadAmps>|<SolarAmps>|<AcAmps>|<BatterySoc>|<BatteryCapacityAh>
            
            if (parts.Length < 10)
                return;

            this.LastReadDtm = DateTimeOffset.Parse(parts[0]);
            this.LastReadUptime = Int32.Parse(parts[1]);

            this.Voltage.UpdateValue(parts[3], this.LastReadDtm);
            this.BatteryAmperage.UpdateValue(parts[4], this.LastReadDtm);
            this.LoadAmperage.UpdateValue(parts[5], this.LastReadDtm);
            this.SolarAmperage.UpdateValue(parts[6], this.LastReadDtm);
            this.AcAmperage.UpdateValue(parts[7], this.LastReadDtm);
            this.BatterySoc.UpdateValue(parts[8], this.LastReadDtm);
            this.BatteryCapacityAh.UpdateValue(parts[9], this.LastReadDtm);

            ParseSuccess = true;
        }
    }
}
