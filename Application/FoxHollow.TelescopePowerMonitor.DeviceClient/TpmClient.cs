using FoxHollow.TelescopePowerMonitor.DeviceClient.Exceptions;
using System;
using System.ComponentModel;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading;
using System.Threading.Tasks;

namespace FoxHollow.TelescopePowerMonitor.DeviceClient
{
    public delegate void TpmLogLineReceived(string line);
    public delegate void TpmPowerChanged(PowerInfo powerInfo);
    public delegate void TpmEnvironmentChanged(EnvironmentInfo environmentInfo);
    public delegate void TpmStatusChanged(StatusInfo statusInfo);

    public class TpmClient : INotifyPropertyChanged
    {
        private SerialPort _serialPort;
        private CancellationTokenSource _cts;
        private bool _connected = false;

        public event TpmLogLineReceived OnLogLine;
        public event TpmPowerChanged OnPowerChanged;
        public event TpmEnvironmentChanged OnEnvironmentChanged;
        public event TpmStatusChanged OnStatusChanged;

        public event PropertyChangedEventHandler PropertyChanged;

        public DateTimeOffset ConnectDtm { get; private set; }
        public string Port { get; private set; } = String.Empty;

        public bool Connected 
        {
            get => _connected;
            private set
            {
                _connected = value;
                OnPropertyChanged();
            }
        }

        public int ConnectedDuration { get; private set; } = 0;
        public int UptimeSeconds { get; private set; } = 0;

        public PowerInfo Power { get; }
        public EnvironmentInfo Environment { get; }
        public StatusInfo Status { get; }
        public EEPROM DeviceConfig { get; }

        public TpmClient()
        {
            this.Power = new PowerInfo();
            this.Environment = new EnvironmentInfo();
            this.Status = new StatusInfo();
            this.DeviceConfig = new EEPROM();

            this.OnLogLine += delegate { };
            this.OnPowerChanged += delegate { };
            this.OnEnvironmentChanged += delegate { };
            this.OnStatusChanged += delegate { };
        }

        public void SetSerialPort(string serialPort)
        {
            Port = serialPort;

            if (!SerialPort.GetPortNames().Where(x => x == Port).Any())
                throw new SerialPortNotFoundException(Port);
        }

        public void Connect()
        {
            if (this.Connected)
                return;

            if (String.IsNullOrWhiteSpace(this.Port))
                throw new ArgumentNullException("Port was not specified");

            this.DeviceConfig.Reset();
            this.Environment.Reset();
            this.Status.Reset();
            this.Power.Reset();

            // Open serial port
            _serialPort = new SerialPort(Port, 115200);
            _serialPort.DataReceived += SerialPortDataReceived;

            try
            {
                Thread.Sleep(200);
                _serialPort.Open();

                _serialPort.ReadExisting();

                OnLogLine("Connected to " + Port);

                this.Connected = true;
                this.ConnectDtm = DateTimeOffset.Now;
                this.PcSetState(true);

                _cts = new CancellationTokenSource();

                this.SetupPingTimer();
            }
            catch (Exception e)
            {
                OnLogLine("Failed to connect to " + Port + ", ERROR: " + e.Message);
            }
        }

        private async void SetupPingTimer()
        {
            while (true)
            {
                try
                {
                    this.SendPing();
                    await Task.Delay(10000, _cts.Token);
                }
                catch (TaskCanceledException)
                {
                    return;
                }
            }
        }

        public void Disconnect()
        {
            if (!this.Connected)
                return;

            if (_serialPort != null && _serialPort.IsOpen)
            {
                _cts.Cancel();

                this.PcSetState(false);

                _serialPort.ReadExisting();
                _serialPort.DataReceived -= SerialPortDataReceived;
                _serialPort.DiscardInBuffer();
                _serialPort.DiscardOutBuffer();

                Thread.Sleep(250);

                try
                {
                    _serialPort.Close();
                }
                catch (IOException ex)
                {
                    this.OnLogLine($"ERROR: {ex.Message}");
                }

                this.ConnectDtm = default;
                this.Connected = false;

                this.DeviceConfig.Reset();
                this.Environment.Reset();
                this.Status.Reset();
                this.Power.Reset();
            }
        }

        private void SerialPortDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (_serialPort == null || !_serialPort.IsOpen || !this.Connected)
                return;

            try
            {
                string line = _serialPort.ReadLine();

                if (line != null)
                {
                    line = line.Replace("\n", String.Empty).Replace("\r", String.Empty);

                    OnLogLine(line);

                    if (this.Power.ParseLogLine(line))
                        OnPowerChanged(this.Power);

                    else if (this.Environment.ParseLogLine(line))
                        OnEnvironmentChanged(this.Environment);

                    else if (this.Status.ParseLogLine(line))
                        OnStatusChanged(this.Status);

                    else if (this.DeviceConfig.ParseLogLine(line)) { }
                        
                }
            }
            catch (Exception ex)
            {
                OnLogLine($"ERROR: {ex.Message}");
            }
        }

        public void ReadEEPROM()
        {
            WriteToDevice("CONFIG");

            DeviceConfig.EepromRead = true;
        }

        private void WriteToDevice(string line)
        {
            if (!this.Connected || _serialPort == null || !_serialPort.IsOpen)
                return;

            _serialPort.Write(line + "\n");
            Thread.Sleep(100);
        }

        public void SaveEEPROM()
        {
            // we don't want to attempt to write the EEPROM if it hasn't first been read
            if (!this.DeviceConfig.EepromRead)
                return;

            this.WriteToDevice($"SET AverageReadingCount {this.DeviceConfig.AverageReadingCount}");
            this.WriteToDevice($"SET UpdateFrequency {this.DeviceConfig.UpdateFrequency}");
            this.WriteToDevice($"SET WriteInterval {this.DeviceConfig.WriteInterval}");
            this.WriteToDevice($"SET VoltageCalibration {this.DeviceConfig.VoltageCalibration}");
            this.WriteToDevice($"SET R1Actual {this.DeviceConfig.R1Actual}");
            this.WriteToDevice($"SET R2Actual {this.DeviceConfig.R2Actual}");
            this.WriteToDevice($"SET AmpDigitalOffset1 {this.DeviceConfig.AmpDigitalOffset1}");
            this.WriteToDevice($"SET AmpDigitalOffset2 {this.DeviceConfig.AmpDigitalOffset2}");
            this.WriteToDevice($"SET AmpDigitalOffset3 {this.DeviceConfig.AmpDigitalOffset3}");
            this.WriteToDevice($"SET TemperatureCalibration {this.DeviceConfig.TemperatureCalibration}");
            this.WriteToDevice($"SET HumidityCalibration {this.DeviceConfig.HumidityCalibration}");
            this.WriteToDevice($"SET TargetHumidity {this.DeviceConfig.TargetHumidity}");
            this.WriteToDevice($"SET HumidityHysterisis {this.DeviceConfig.HumidityHysterisis}");
            this.WriteToDevice($"SET AcBackupPoint {this.DeviceConfig.AcBackupPoint}");
            this.WriteToDevice($"SET AcRecoveredPoint {this.DeviceConfig.AcRecoveredPoint}");
            this.WriteToDevice($"SET BatteryCapacityAh {this.DeviceConfig.BatteryCapacityAh}");
            this.WriteToDevice($"SET BatteryEndingAmps {this.DeviceConfig.BatteryEndingAmps}");
            this.WriteToDevice($"SET BatteryAbsorbVoltage {this.DeviceConfig.BatteryAbsorbVoltage}");

            WriteToDevice("SAVE");
        }

        public void PauseReadings()
        {
            WriteToDevice("PAUSE");
            _serialPort.ReadExisting();
        }

        public void ResumeReadings()
            => this.WriteToDevice("RESUME");

        public void ResetEeprom()
        { 
            this.WriteToDevice("RESET EEPROM");
            this.ReadEEPROM();
        }

        public void SendPing()
            => this.WriteToDevice("PING");

        public void PcSetState(bool state)
            => this.WriteToDevice($"PC {GetOnOffState(state)}");

        public void AutoDehumSetState(bool state)
            => this.WriteToDevice($"DEHUM {GetOnOffState(state)}");

        public void AutoDehumToggleState()
            => this.AutoDehumSetState(!this.Status.DehumEnabled);

        private void SetRelayState(string relay, bool state)
            => this.WriteToDevice($"PWR {GetOnOffState(state)} {relay}");

        public void TelescopePwrState(bool state)
            => this.SetRelayState("TELESCOPE", state);

        public void TelescopeToggleState()
            => this.TelescopePwrState(!this.Status.TelescopeOutState);

        public void DehumidifierPwrState(bool state)
            => this.SetRelayState("DEHUM", state);

        public void DehumidifierToggleState()
            => this.DehumidifierPwrState(!this.Status.DehumOutState);

        public void Aux1PwrState(bool state)
            => this.SetRelayState("AUX1", state);

        public void Aux1ToggleState()
            => this.Aux1PwrState(!this.Status.Aux1OutState);

        public void AcPwrState(bool state)
            => this.SetRelayState("AC", state);

        public void AcToggleState()
            => this.AcPwrState(!this.Status.AcInState);

        public void UpdateTime()
        {
            string timeStr = DateTimeOffset.UtcNow.AddMilliseconds(250).ToString("yyyy-MM-ddTHH:mm:ss");
            this.WriteToDevice($"SET TIME {timeStr}");
        }

        string GetOnOffState(bool status) => (status ? "ON" : "OFF");

        protected void OnPropertyChanged([CallerMemberName] string name = null)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
    }
}
