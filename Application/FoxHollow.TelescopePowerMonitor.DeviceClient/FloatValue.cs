using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Text;

namespace FoxHollow.TelescopePowerMonitor.DeviceClient
{
    public class FloatValue : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        private float _current = 0.0F;
        private float _minimum = 0.0F;
        private float _maximum = 0.0F;

        private DateTimeOffset _currentDtm;
        private DateTimeOffset _minimumDtm;
        private DateTimeOffset _maximumDtm;

        private bool _valid = false;

        public bool Valid
        {
            get => _valid;
            internal set
            {
                _valid = value;
                OnPropertyChanged();
            }
        }

        public float Current
        {
            get => _current;
            private set
            {
                _current = value;
                OnPropertyChanged();
            }
        }
        public float Minimum
        {
            get => _minimum;
            private set
            {
                _minimum = value;
                OnPropertyChanged();
            }
        }
        public float Maximum
        {
            get => _maximum;
            private set
            {
                _maximum = value;
                OnPropertyChanged();
            }
        }

        public DateTimeOffset CurrentDtm 
        { 
            get => _currentDtm;
            private set
            {
                _currentDtm = value;
                OnPropertyChanged();
            }
        }
        public DateTimeOffset MinimumDtm 
        { 
            get => _minimumDtm; 
            private set
            {
                _minimumDtm = value;
                OnPropertyChanged();
            }
        }
        public DateTimeOffset MaximumDtm 
        { 
            get => _maximumDtm; 
            private set
            {
                _maximumDtm = value;
                OnPropertyChanged();
            }
        }

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

            this.Valid = false;
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

        protected void OnPropertyChanged([CallerMemberName] string name = null)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
    }
}
