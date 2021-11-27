using System;
using System.Collections.Generic;
using System.Text;

namespace FoxHollow.TelescopePowerMonitor.DeviceClient
{
    public class FloatValue
    {
        public float Current { get; private set; } = 0.0F;
        public float Minimum { get; private set; } = 0.0F;
        public float Maximum { get; private set; } = 0.0F;

        public DateTimeOffset CurrentDtm { get; private set; }
        public DateTimeOffset MinimumDtm { get; private set; }
        public DateTimeOffset MaximumDtm { get; private set; }

        internal void UpdateValue(string floatStr, DateTimeOffset timestamp)
            => UpdateValue(Single.Parse(floatStr), timestamp);

        internal void Reset()
        {
            this.Current = 0.0F;
            this.Minimum = 0.0F;
            this.Maximum = 0.0F;

            this.CurrentDtm = default;
            this.MinimumDtm = default;
            this.MaximumDtm = default;
        }

        internal void UpdateValue(float value, DateTimeOffset timestamp)
        {
            this.Current = value;
            this.CurrentDtm = timestamp;
            
            if (this.Current > this.Maximum)
            {
                this.Maximum = this.Current;
                this.MaximumDtm = timestamp;
            }

            if (this.Current < this.Minimum || this.Minimum == 0F)
            {
                this.Minimum = this.Current;
                this.MinimumDtm = timestamp;
            }
        }
    }
}
