using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text;

namespace FoxHollow.TelescopePowerMonitor.DeviceClient
{
    public class StatusInfo : IDeviceLogType, INotifyPropertyChanged
    {
        private DateTimeOffset _lastReadDtm;
        private int _lastReadUptime;

        private bool _valid;

        private int _bytesFreeMem;
        private bool _dehumEnabled;
        private bool _telescopeOutState;
        private bool _dehumOutState;
        private bool _aux1OutState;
        private bool _acInState;
        private int _batteryCurrentStateSeconds;
        private int _dehumCurrentStateSeconds;
        private bool _pcConnected;
        private int _lastPingSeconds;

        private float _timeDelta = 0.0F;

        public DateTimeOffset LastReadDtm
        {
            get => _lastReadDtm;
            set
            {
                _lastReadDtm = value;
                OnPropertyChanged();
            }
        }
        public int LastReadUptime
        {
            get => _lastReadUptime;
            set
            {
                _lastReadUptime = value;
                OnPropertyChanged();
            }
        }
        public bool Valid
        {
            get => _valid;
            set
            {
                _valid = value;
                OnPropertyChanged();
            }
        }
        public int BytesFreeMem
        {
            get => _bytesFreeMem;
            set
            {
                _bytesFreeMem = value;
                OnPropertyChanged();
            }
        }
        public bool DehumEnabled
        {
            get => _dehumEnabled;
            set
            {
                _dehumEnabled = value;
                OnPropertyChanged();
            }
        }
        public bool TelescopeOutState
        {
            get => _telescopeOutState;
            set
            {
                _telescopeOutState = value;
                OnPropertyChanged();
            }
        }
        public bool DehumOutState
        {
            get => _dehumOutState;
            set
            {
                _dehumOutState = value;
                OnPropertyChanged();
            }
        }
        public bool Aux1OutState
        {
            get => _aux1OutState;
            set
            {
                _aux1OutState = value;
                OnPropertyChanged();
            }
        }
        public bool AcInState
        {
            get => _acInState;
            set
            {
                _acInState = value;
                OnPropertyChanged();
            }
        }
        public int BatteryCurrentStateSeconds
        {
            get => _batteryCurrentStateSeconds;
            set
            {
                _batteryCurrentStateSeconds = value;
                OnPropertyChanged();
            }
        }

        public int DehumCurrentStateSeconds
        {
            get => _dehumCurrentStateSeconds;
            set
            {
                _dehumCurrentStateSeconds = value;
                OnPropertyChanged();
            }
        }
        public bool PcConnected
        {
            get => _pcConnected;
            set
            {
                _pcConnected = value;
                OnPropertyChanged();
            }
        }
        public int LastPingSeconds
        {
            get => _lastPingSeconds;
            set
            {
                _lastPingSeconds = value;
                OnPropertyChanged();
            }
        }
        public float TimeDelta
        {
            get => _timeDelta;
            set
            {
                _timeDelta = value;
                OnPropertyChanged();
            }
        }


        public event PropertyChangedEventHandler PropertyChanged;

        internal bool ParseLogLine(string inputLine)
        {
            string[] parts = inputLine.Split('|');

            //    0             1         2           3            4
            // <ISO_DTM>|<UptimeSeconds>|STAT|<BytesFreeMem>|<DehumEnabled>|
            
            //           5                6               7            8      
            // <TelescopeOutState>|<DehumOutState>|<Aux1OutState>|<AcInState>|

            //             9                             10                 11               12
            // <BatteryCurrentStateSeconds>|<DehumCurrentStateSeconds>|<PcConnected>|<LastPingSeconds>

            // TODO: fix
            if (parts.Length < 5)
                return false;

            if (!parts[2].Equals("STAT", StringComparison.InvariantCultureIgnoreCase))
                return false;

            DateTimeOffset nowUtc = DateTimeOffset.UtcNow;

            this.LastReadDtm = DateTimeOffset.Parse(parts[0], null, DateTimeStyles.AssumeUniversal);
            this.LastReadUptime = Int32.Parse(parts[1]);

            this.BytesFreeMem = Int32.Parse(parts[3]);
            this.DehumEnabled = ParseBool(parts[4]);
            this.TelescopeOutState = ParseBool(parts[5]);
            this.DehumOutState = ParseBool(parts[6]);
            this.Aux1OutState = ParseBool(parts[7]);
            this.AcInState = ParseBool(parts[8]);
            this.BatteryCurrentStateSeconds = Int32.Parse(parts[9]);
            this.DehumCurrentStateSeconds = Int32.Parse(parts[10]);
            //this.PcConnected = ParseBool(parts[11]);
            this.LastPingSeconds = Int32.Parse(parts[11]);

            TimeSpan delta = this.LastReadDtm - nowUtc;
            delta.Add(TimeSpan.FromMilliseconds(-250));

            // Calculate the time delta between the PC and the controller
            this.TimeDelta = (float)delta.TotalSeconds;

            this.Valid = true;

            return true;
        }

        private static bool ParseBool(string input)
            => input == "1";

        internal void Reset()
        {
            this.LastReadDtm = default;
            this.LastReadUptime = 0;

            this.BytesFreeMem = 0;
            this.DehumEnabled = false;
            this.TelescopeOutState = false;
            this.DehumOutState = false;
            this.Aux1OutState = false;
            this.AcInState = false;
            this.BatteryCurrentStateSeconds = 0;
            this.DehumCurrentStateSeconds = 0;
            this.PcConnected = false;
            this.LastPingSeconds = 0;

            this.Valid = false;
        }

        protected void OnPropertyChanged([CallerMemberName] string name = null)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
    }
}
