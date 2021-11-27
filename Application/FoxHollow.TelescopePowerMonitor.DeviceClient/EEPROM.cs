using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace FoxHollow.TelescopePowerMonitor.DeviceClient
{
    public class EEPROM : INotifyPropertyChanged, IDeviceLogType
    {
        private DateTimeOffset _lastReadDtm;
        private int _lastReadUptime;

        private int _AverageReadingCount;
        private float _UpdateFrequency;
        private float _WriteInterval;
        private float _VoltageCalibration;
        private int _R1Actual;
        private int _R2Actual;
        private int _AmpDigitalOffset;

        private bool _eepromRead = false;

        public event PropertyChangedEventHandler PropertyChanged;

        public bool ParseSuccess { get; private set; } = false;

        public int LastReadUptime
        {
            get => _lastReadUptime;
            set
            {
                _lastReadUptime = value;
                OnPropertyChanged();
            }
        }

        public DateTimeOffset LastReadDtm
        {
            get => _lastReadDtm;
            set
            {
                _lastReadDtm = value;
                OnPropertyChanged();
            }
        }

        public bool EepromRead
        {
            get 
            {
                return _eepromRead;
            }
            set
            {
                _eepromRead = value;
                OnPropertyChanged();
            }
        }

        public int AverageReadingCount
        {
            get => _AverageReadingCount;
            set
            {
                _AverageReadingCount = value;
                OnPropertyChanged();
            }
        }
        public float UpdateFrequency
        {
            get => _UpdateFrequency;
            set
            {
                _UpdateFrequency = value;
                OnPropertyChanged();
            }
        }
        public float WriteInterval
        {
            get => _WriteInterval;
            set
            {
                _WriteInterval = value;
                OnPropertyChanged();
            }
        }
        public float VoltageCalibration
        {
            get => _VoltageCalibration;
            set
            {
                _VoltageCalibration = value;
                OnPropertyChanged();
            }
        }
        public int R1Actual
        {
            get => _R1Actual;
            set
            {
                _R1Actual = value;
                OnPropertyChanged();
            }
        }
        public int R2Actual
        {
            get => _R2Actual;
            set
            {
                _R2Actual = value;
                OnPropertyChanged();
            }
        }
        public int AmpDigitalOffset
        {
            get => _AmpDigitalOffset;
            set
            {
                _AmpDigitalOffset = value;
                OnPropertyChanged();
            }
        }

        internal void ParseLogLine(string line)
        {
            //         0             1          2           3              4
            // ex: <ISO_DTM>|<UptimeSeconds>|CONFIG|AverageReadingCount|<value>
            string[] parts = line.Split('|');

            if (parts.Length < 5)
                return;

            if (!parts[2].Equals("CONFIG", StringComparison.InvariantCultureIgnoreCase))
                return;

            string variable = parts[3];
            string strValue = parts[4];

            if (variable.Equals("AverageReadingCount", StringComparison.InvariantCultureIgnoreCase))
                this.AverageReadingCount = Int32.Parse(strValue);
            
            else if (variable.Equals("UpdateFrequency", StringComparison.InvariantCultureIgnoreCase)) 
                this.UpdateFrequency = Single.Parse(strValue);

            else if (variable.Equals("WriteInterval", StringComparison.InvariantCultureIgnoreCase)) 
                this.WriteInterval = Single.Parse(strValue);

            else if (variable.Equals("VoltageCalibration", StringComparison.InvariantCultureIgnoreCase)) 
                this.VoltageCalibration = Single.Parse(strValue);

            else if (variable.Equals("R1Actual", StringComparison.InvariantCultureIgnoreCase)) 
                this.R1Actual = Int32.Parse(strValue);

            else if (variable.Equals("R2Actual", StringComparison.InvariantCultureIgnoreCase)) 
                this.R2Actual = Int32.Parse(strValue);

            else if (variable.Equals("AmpDigitalOffset", StringComparison.InvariantCultureIgnoreCase)) 
                this.AmpDigitalOffset = Int32.Parse(strValue);


            this.ParseSuccess = true;
        }

        protected void OnPropertyChanged([CallerMemberName] string name = null)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
    }
}
