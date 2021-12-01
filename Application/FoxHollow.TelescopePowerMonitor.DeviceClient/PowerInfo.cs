using System;
using System.Collections.Generic;
using System.Globalization;
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

        internal bool ParseLogLine(string inputLine)
        {
            string[] parts = inputLine.Split('|');

            //    0             1         2      3           4            5          6         7            8                 9
            // <ISO_DTM>|<UptimeSeconds>|PWR|<Voltage>|<BatteryAmps>|<LoadAmps>|<SolarAmps>|<AcAmps>|<BatterySoc>|<BatteryCapacityAh>
            
            if (parts.Length < 10)
                return false;

            if (!parts[2].Equals("PWR", StringComparison.InvariantCultureIgnoreCase))
                return false;

            this.LastReadDtm = DateTimeOffset.Parse(parts[0], null, DateTimeStyles.AssumeUniversal);
            this.LastReadUptime = Int32.Parse(parts[1]);

            this.Voltage.UpdateValue(parts[3], this.LastReadDtm);
            this.BatteryAmperage.UpdateValue(parts[4], this.LastReadDtm);
            this.LoadAmperage.UpdateValue(parts[5], this.LastReadDtm);
            this.SolarAmperage.UpdateValue(parts[6], this.LastReadDtm);
            this.AcAmperage.UpdateValue(parts[7], this.LastReadDtm);
            this.BatterySoc.UpdateValue(parts[8], this.LastReadDtm);
            this.BatteryCapacityAh.UpdateValue(parts[9], this.LastReadDtm);

            this.Voltage.Valid = true;
            this.BatteryAmperage.Valid = true;
            this.LoadAmperage.Valid = true;
            this.SolarAmperage.Valid = true;
            this.AcAmperage.Valid = true;
            this.BatterySoc.Valid = true;
            this.BatteryCapacityAh.Valid = true;


            return true;
        }

        internal void Reset()
        {
            this.LastReadDtm = default;
            this.LastReadUptime = 0;

            this.Voltage.Reset();
            this.BatteryAmperage.Reset();
            this.LoadAmperage.Reset();
            this.SolarAmperage.Reset();
            this.AcAmperage.Reset();
            this.BatterySoc.Reset();
            this.BatteryCapacityAh.Reset();
        }
    }
}
