using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Globalization;
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

        private int _averageReadingCount;
        private float _updateFrequency;
        private float _writeInterval;
        private float _voltageCalibration;
        private int _r1Actual;
        private int _r2Actual;
        private int _ampDigitalOffset1;
        private int _ampDigitalOffset2;
        private int _ampDigitalOffset3;
        private float _temperatureCalibration;
        private float _humidityCalibration;
        private int _targetHumidity;
        private int _humidityHysterisis;
        private int _acBackupPoint;
        private int _acRecoveredPoint;
        private int _batteryCapacityAh;
        private float _batteryEndingAmps;
        private float _batteryAbsorbVoltage;

        private bool _eepromRead = false;

        public event PropertyChangedEventHandler PropertyChanged;

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
            get => _averageReadingCount;
            set
            {
                _averageReadingCount = value;
                OnPropertyChanged();
            }
        }
        public float UpdateFrequency
        {
            get => _updateFrequency;
            set
            {
                _updateFrequency = value;
                OnPropertyChanged();
            }
        }
        public float WriteInterval
        {
            get => _writeInterval;
            set
            {
                _writeInterval = value;
                OnPropertyChanged();
            }
        }
        public float VoltageCalibration
        {
            get => _voltageCalibration;
            set
            {
                _voltageCalibration = value;
                OnPropertyChanged();
            }
        }
        public int R1Actual
        {
            get => _r1Actual;
            set
            {
                _r1Actual = value;
                OnPropertyChanged();
            }
        }
        public int R2Actual
        {
            get => _r2Actual;
            set
            {
                _r2Actual = value;
                OnPropertyChanged();
            }
        }
        public int AmpDigitalOffset1
        {
            get => _ampDigitalOffset1;
            set
            {
                _ampDigitalOffset1 = value;
                OnPropertyChanged();
            }
        }
        public int AmpDigitalOffset2
        {
            get => _ampDigitalOffset2;
            set
            {
                _ampDigitalOffset2 = value;
                OnPropertyChanged();
            }
        }
        public int AmpDigitalOffset3
        {
            get => _ampDigitalOffset3;
            set
            {
                _ampDigitalOffset3 = value;
                OnPropertyChanged();
            }
        }
        public float TemperatureCalibration
        {
            get => _temperatureCalibration;
            set
            {
                _temperatureCalibration = value;
                OnPropertyChanged();
            }
        }
        public float HumidityCalibration
        {
            get => _humidityCalibration;
            set
            {
                _humidityCalibration = value;
                OnPropertyChanged();
            }
        }
        public int TargetHumidity
        {
            get => _targetHumidity;
            set
            {
                _targetHumidity = value;
                OnPropertyChanged();
            }
        }
        public int HumidityHysterisis
        {
            get => _humidityHysterisis;
            set
            {
                _humidityHysterisis = value;
                OnPropertyChanged();
            }
        }
        public int AcBackupPoint
        {
            get => _acBackupPoint;
            set
            {
                _acBackupPoint = value;
                OnPropertyChanged();
            }
        }
        public int AcRecoveredPoint
        {
            get => _acRecoveredPoint;
            set
            {
                _acRecoveredPoint = value;
                OnPropertyChanged();
            }
        }
        public int BatteryCapacityAh
        {
            get => _batteryCapacityAh;
            set
            {
                _batteryCapacityAh = value;
                OnPropertyChanged();
            }
        }
        public float BatteryEndingAmps
        {
            get => _batteryEndingAmps;
            set
            {
                _batteryEndingAmps = value;
                OnPropertyChanged();
            }
        }
        public float BatteryAbsorbVoltage
        {
            get => _batteryAbsorbVoltage;
            set
            {
                _batteryAbsorbVoltage = value;
                OnPropertyChanged();
            }
        }

        
        public EEPROM()
        {
            this.PropertyChanged += ValidateProperty;
        }

        internal bool ParseLogLine(string line)
        {
            //         0             1          2           3              4
            // ex: <ISO_DTM>|<UptimeSeconds>|CONFIG|AverageReadingCount|<value>
            string[] parts = line.Split('|');

            if (parts.Length < 5)
                return false;

            if (!parts[2].Equals("CONFIG", StringComparison.InvariantCultureIgnoreCase))
                return false;

            this.LastReadDtm = DateTimeOffset.Parse(parts[0], null, DateTimeStyles.AssumeUniversal);
            this.LastReadUptime = Int32.Parse(parts[1]);

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

            else if (variable.Equals("AmpDigitalOffset1", StringComparison.InvariantCultureIgnoreCase)) 
                this.AmpDigitalOffset1 = Int32.Parse(strValue);
            
            else if (variable.Equals("AmpDigitalOffset2", StringComparison.InvariantCultureIgnoreCase)) 
                this.AmpDigitalOffset2 = Int32.Parse(strValue);
            
            else if (variable.Equals("AmpDigitalOffset3", StringComparison.InvariantCultureIgnoreCase)) 
                this.AmpDigitalOffset3 = Int32.Parse(strValue);

            else if (variable.Equals("TemperatureCalibration", StringComparison.InvariantCultureIgnoreCase))
                this.TemperatureCalibration = Single.Parse(strValue);

            else if (variable.Equals("HumidityCalibration", StringComparison.InvariantCultureIgnoreCase))
                this.HumidityCalibration = Single.Parse(strValue);

            else if (variable.Equals("TargetHumidity", StringComparison.InvariantCultureIgnoreCase))
                this.TargetHumidity = Int32.Parse(strValue);

            else if (variable.Equals("HumidityHysterisis", StringComparison.InvariantCultureIgnoreCase))
                this.HumidityHysterisis = Int32.Parse(strValue);

            else if (variable.Equals("AcBackupPoint", StringComparison.InvariantCultureIgnoreCase))
                this.AcBackupPoint = Int32.Parse(strValue);

            else if (variable.Equals("AcRecoveredPoint", StringComparison.InvariantCultureIgnoreCase))
                this.AcRecoveredPoint = Int32.Parse(strValue);

            else if (variable.Equals("BatteryCapacityAh", StringComparison.InvariantCultureIgnoreCase))
                this.BatteryCapacityAh = Int32.Parse(strValue);

            else if (variable.Equals("BatteryEndingAmps", StringComparison.InvariantCultureIgnoreCase))
                this.BatteryEndingAmps = Single.Parse(strValue);

            else if (variable.Equals("BatteryAbsorbVoltage", StringComparison.InvariantCultureIgnoreCase))
                this.BatteryAbsorbVoltage = Single.Parse(strValue);

            else 
                return false;

            return true;
        }

        internal void Reset()
        {
            LastReadDtm = default;
            LastReadUptime = 0;

            EepromRead = false;

            AverageReadingCount = 0;
            UpdateFrequency = 0;
            WriteInterval = 0;
            VoltageCalibration = 0;
            R1Actual = 0;
            R2Actual = 0;
            AmpDigitalOffset1 = 0;
            AmpDigitalOffset2 = 0;
            AmpDigitalOffset3 = 0;
            TemperatureCalibration = 0;
            HumidityCalibration = 0;
            TargetHumidity = 0;
            HumidityHysterisis = 0;
            AcBackupPoint = 0;
            AcRecoveredPoint = 0;
            BatteryCapacityAh = 0;
            BatteryEndingAmps = 0;
            BatteryAbsorbVoltage = 0;

        }

        protected void OnPropertyChanged([CallerMemberName] string name = null)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));

        private void ValidateProperty(object sender, System.ComponentModel.PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(this.AcBackupPoint))
            {
                if (this.AcBackupPoint > this.AcRecoveredPoint)
                    this.AcRecoveredPoint = this.AcBackupPoint + 1;
            }

            if (e.PropertyName == nameof(this.AcRecoveredPoint))
            {
                if (this.AcRecoveredPoint < this.AcBackupPoint)
                    this.AcBackupPoint = this.AcRecoveredPoint - 1;
            }
        }
    }
}
