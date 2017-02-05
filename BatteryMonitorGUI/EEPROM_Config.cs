using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BatteryMonitorGUI
{
    public class EEPROM_Config : INotifyPropertyChanged
    {
        private int _AverageReadingCount;
        private float _UpdateFrequency;
        private float _WriteInterval;
        private float _VoltageCalibration;
        private int _R1Actual;
        private int _R2Actual;
        private int _AmpDigitalOffset;

        private bool _EEPROMRead = false;

        public bool EEPROMRead
        {
            get
            {
                return _EEPROMRead;
            }
            set
            {
                _EEPROMRead = value;

                if (PropertyChanged != null)
                    PropertyChanged(this, new PropertyChangedEventArgs("EEPROMRead"));
            }
        }

        public int AverageReadingCount
        {
            get
            {
                return _AverageReadingCount;
            }
            set
            {
                _AverageReadingCount = value;

                if (PropertyChanged != null)
                    PropertyChanged(this, new PropertyChangedEventArgs("AverageReadingCount"));
            }
        }
        public float UpdateFrequency
        {
            get
            {
                return _UpdateFrequency;
            }
            set
            {
                _UpdateFrequency = value;

                if (PropertyChanged != null)
                    PropertyChanged(this, new PropertyChangedEventArgs("UpdateFrequency"));
            }
        }
        public float WriteInterval
        {
            get
            {
                return _WriteInterval;
            }
            set
            {
                _WriteInterval = value;

                if (PropertyChanged != null)
                    PropertyChanged(this, new PropertyChangedEventArgs("WriteInterval"));
            }
        }
        public float VoltageCalibration
        {
            get
            {
                return _VoltageCalibration;
            }
            set
            {
                _VoltageCalibration = value;

                if (PropertyChanged != null)
                    PropertyChanged(this, new PropertyChangedEventArgs("VoltageCalibration"));
            }
        }
        public int R1Actual
        {
            get
            {
                return _R1Actual;
            }
            set
            {
                _R1Actual = value;

                if (PropertyChanged != null)
                    PropertyChanged(this, new PropertyChangedEventArgs("R1Actual"));
            }
        }
        public int R2Actual
        {
            get
            {
                return _R2Actual;
            }
            set
            {
                _R2Actual = value;

                if (PropertyChanged != null)
                    PropertyChanged(this, new PropertyChangedEventArgs("R2Actual"));
            }
        }
        public int AmpDigitalOffset
        {
            get
            {
                return _AmpDigitalOffset;
            }
            set
            {
                _AmpDigitalOffset = value;

                if (PropertyChanged != null)
                    PropertyChanged(this, new PropertyChangedEventArgs("AmpDigitalOffset"));
            }
        }

        public void parseConfig(string configLine)
        {
            string trimmedLine = configLine.Replace("CONFIG|", "");

            string[] parts = trimmedLine.Split(':');

            string variable = parts[0];
            string value = parts[1];

            if (variable == "AverageReadingCount")
                AverageReadingCount = Int32.Parse(value);
            else if (variable == "UpdateFrequency")
                UpdateFrequency = Single.Parse(value);
            else if (variable == "WriteInterval")
                WriteInterval = Single.Parse(value);
            else if (variable == "VoltageCalibration")
                VoltageCalibration = Single.Parse(value);
            else if (variable == "R1Actual")
                R1Actual = Int32.Parse(value);
            else if (variable == "R2Actual")
                R2Actual = Int32.Parse(value);
            else if (variable == "AmpDigitalOffset")
                AmpDigitalOffset = Int32.Parse(value);
        }

        public EEPROM_Config()
        {

        }

        public event PropertyChangedEventHandler PropertyChanged;
    }
}
