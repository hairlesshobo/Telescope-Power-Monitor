using Microsoft.WindowsAPICodePack.Dialogs;
using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using static BatteryMonitorGUI.Properties.Settings;

namespace BatteryMonitorGUI
{
    public class VoltageInfo
    {
        public int UptimeSeconds { get; set; } = 0;

        public float VoltageAvg { get; set; } = 0.0F;
        public float VoltageMin { get; set; } = 0.0F;
        public float VoltageMax { get; set; } = 0.0F;

        public float AmperageAvg { get; set; } = 0.0F;
        public float AmperageMin { get; set; } = 0.0F;
        public float AmperageMax { get; set; } = 0.0F;

        public bool ParseSuccess { get; set; } = false;

        public VoltageInfo (string inputLine)
        {
            string[] parts = inputLine.Split('|');

            if (parts.Length < 7)
                return;

            UptimeSeconds = Int32.Parse(parts[0]);

            VoltageAvg = Single.Parse(parts[1]);
            VoltageMin = Single.Parse(parts[2]);
            VoltageMax = Single.Parse(parts[3]);

            AmperageAvg = Single.Parse(parts[4]);
            AmperageMin = Single.Parse(parts[5]);
            AmperageMax = Single.Parse(parts[6]);

            ParseSuccess = true;
        }
    }
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private bool _connected = false;
        private string _comPort = "";
        private SerialPort _serialPort = null;
        private StreamWriter _logFile = null;

        private float _minVoltage;
        private float _maxVoltage;
        private float _minAmperage;
        private float _maxAmperage;
        
        public EEPROM_Config deviceConfig { get; set; } = new EEPROM_Config();

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            button_Connect.IsEnabled = false;

            SetDefaultLogDirectory();

            uptimeStatusBlock.Text = "---";
            mainWindow.Topmost = Default.alwaysOnTop;

            statusBar_SetDisconnected();
            refreshPorts();
            clearStatusValues();

            setLastComPort();

            attachValueWatchers();
        }

        private void SetDefaultLogDirectory()
        {
            if (Default.logDirectory == "")
            {
                Default.logDirectory =
                    Path.Combine(
                        Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
                        "TelescopeBatteryMonitor"
                    );

                Default.Save();
            }
        }

        private void attachValueWatchers()
        {
            Default.PropertyChanged += Default_PropertyChanged1; ;
        }

        private void Default_PropertyChanged1(object sender, System.ComponentModel.PropertyChangedEventArgs e)
        {
            if (e.PropertyName == "watchVoltage")
            {
                if (Default.watchVoltage == Default.minVoltageRange)
                    Default.warningVoltage = Default.minVoltageRange;
                else
                {
                    if (Default.warningVoltage >= Default.watchVoltage)
                        Default.warningVoltage = Default.watchVoltage - 0.1F;
                }
            }

            if (e.PropertyName == "warningVoltage")
            {
                if (Default.warningVoltage == Default.maxVoltageRange)
                    Default.watchVoltage = Default.maxVoltageRange;
                else
                {
                    if (Default.warningVoltage >= Default.watchVoltage)
                        Default.watchVoltage = Default.warningVoltage + 0.1F;
                }
            }

            if (e.PropertyName == "alwaysOnTop")
                mainWindow.Topmost = Default.alwaysOnTop;

            if (e.PropertyName == "logDirectory")
            {
                if (_logFile != null)
                {
                    CloseLogFile();
                    OpenLogFile();
                }
            }

            Default.Save();
        }

        private void setLastComPort()
        {
            if (Default.lastComPort != "")
            {
                if (((List<string>)comPortList.ItemsSource).Contains(Default.lastComPort))
                {
                    comPortList.SelectedItem = Default.lastComPort;

                    if (Default.autoConnectLast)
                        Connect(Default.lastComPort);
                }
            }
        }

        private void refreshPortsButton_Click(object sender, RoutedEventArgs e)
        {
            refreshPorts();
        }

        private void refreshPorts()
        {
            comPortList.ItemsSource = new List<string>() { "Loading.." };

            List<string> comPorts = SerialPort.GetPortNames().ToList();

            comPortList.ItemsSource = comPorts;
        }

        private void connectButton_Click(object sender, RoutedEventArgs e)
        {
            if (!_connected)
            {
                if (comPortList.SelectedItem == null)
                {
                    MessageBox.Show("You must select a COM port to connect to");
                    return;
                }

                Connect(comPortList.SelectedItem.ToString());
            }
            else
                Disconnect();
        }

        private void Connect(string Port)
        {
            if (_connected)
                return;

            if (!SerialPort.GetPortNames().Where(x => x == Port).Any())
            {
                MessageBox.Show("ERROR: Could not find " + Port + " port!");
                return;
            }

            _comPort = Port;
            _minVoltage = 0.0F;
            _maxVoltage = 0.0F;
            _minAmperage = 0.0F;
            _maxAmperage = 0.0F;

            OpenLogFile();

            // Open serial port
            _serialPort = new SerialPort(Port, 9600);
            _serialPort.DataReceived += _serialPort_DataReceived;

            try
            {
                Thread.Sleep(200);
                _serialPort.Open();

                _serialPort.ReadExisting();

                WriteLogLine("Connected to " + _comPort);

                _connected = true;

                statusBar_SetConnected();
                controls_disable();
                button_Connect.Content = "Disconnect";

                Default.lastComPort = _comPort;
                Default.Save();

                uptimeStatusBlock.Foreground = Brushes.DarkGray;

                SetDisplayColors(Brushes.DarkGray);
            }
            catch (Exception e)
            {
                WriteLogLine("Failed to connect to " + _comPort + ", ERROR: " + e.Message);
                CloseLogFile();
            }
        }

        private void EnableSerialDataProcessing()
        {
            _serialPort.DataReceived += _serialPort_DataReceived;
        }

        private void DisableSerialDataProcessing()
        {
            _serialPort.DataReceived -= _serialPort_DataReceived;
        }

        private void Disconnect()
        {
            if (!_connected)
                return;

            if (_serialPort != null)
            {
                if (_serialPort.IsOpen)
                {
                    _serialPort.DataReceived -= _serialPort_DataReceived;
                    _serialPort.ReadExisting();
                    _serialPort.DiscardInBuffer();
                    _serialPort.DiscardOutBuffer();

                    Thread.Sleep(200);
                    _serialPort.Close();
                }
            }

            _connected = false;
            statusBar_SetDisconnected();
            controls_enable();
            button_Connect.Content = "Connect";

            clearStatusValues();

            CloseLogFile();
        }

        private void OpenLogFile()
        {
            // Create log directory
            if (!Directory.Exists(Default.logDirectory))
                Directory.CreateDirectory(Default.logDirectory);

            // Open log file
            string logFileName = "BatteryLog_" + DateTime.Now.ToString("yyyy-MM-dd") + ".txt";

            _logFile = new StreamWriter(Path.Combine(Default.logDirectory, logFileName), true);
            _logFile.AutoFlush = true;

            WriteLogLine("Log file opened");
        }

        private void CloseLogFile()
        {
            WriteLogLine("Log file closed");
            _logFile.Flush();
            _logFile.Close();

            _logFile = null;
        }

        private void WriteLogLine(string line)
        {
            if (_logFile != null)
                _logFile.WriteLine(FormatLogLine(line));
        }

        private void _serialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (_serialPort == null)
                return;

            if (!_serialPort.IsOpen)
                return;

            try
            {
                string line = _serialPort.ReadLine();

                if (line != null)
                {
                    line = line.Replace("\n", String.Empty);
                    line = line.Replace("\r", String.Empty);

                    string logLine = FormatLogLine(line);

                    AppendLog(logLine);

                    if (line.StartsWith("CONFIG"))
                        deviceConfig.parseConfig(line);
                    else
                    {
                        VoltageInfo info = new VoltageInfo(line);

                        if (info.ParseSuccess)
                            SetVoltage(info);
                    }
                }
            }
            catch (Exception)
            {
                return;
            }
        }

        private string FormatLogLine(string line)
        {
            return DateTime.Now.ToString("MM/dd/yyyy HH:mm:ss") + " // " + line;
        }

        public void AppendLog(string value)
        {
            if (!Dispatcher.CheckAccess())
            {
                this.Dispatcher.Invoke(() =>
                {
                    AppendLog(value);
                });
                
                return;
            }

            serialLogBlock.AppendText(value + Environment.NewLine);
            serialLogBlock.ScrollToEnd();

            if (_logFile != null)
                _logFile.WriteLine(value);
        }

        public void clearStatusValues()
        {
            voltageAvgBlock.Text = "---- v";
            voltageMinBlock.Text = "---- v";
            voltageMaxBlock.Text = "---- v";

            amperageAvgBlock.Text = "---- A";
            amperageMaxBlock.Text = "---- A";
            amperageMinBlock.Text = "---- A";

            uptimeStatusBlock.Text = "---";

            uptimeStatusBlock.Foreground = Brushes.Gray;

            SetDisplayColors(Brushes.Gray);
        }

        public void SetVoltage(VoltageInfo info)
        {
            if (!Dispatcher.CheckAccess())
            {
                this.Dispatcher.Invoke(() =>
                {
                    SetVoltage(info);
                });

                return;
            }

            uptimeStatusBlock.Text = FormatUptime(info.UptimeSeconds);

            voltageAvgBlock.Text = info.VoltageAvg.ToString("0.00") + " v";

            if (Default.showLiveMinMax ||
                (info.VoltageMin < _minVoltage || _minVoltage == 0.0F))
                _minVoltage = info.VoltageMin;

            if (Default.showLiveMinMax || 
                (info.VoltageMax > _maxVoltage || _maxVoltage == 0.0F))
                _maxVoltage = info.VoltageMax;

            voltageMinBlock.Text = _minVoltage.ToString("0.00") + " v";
            voltageMaxBlock.Text = _maxVoltage.ToString("0.00") + " v";

            if (Default.showLiveMinMax || 
                (info.AmperageMin < _minAmperage || _minAmperage == 0.0F))
                _minAmperage = info.AmperageMin;

            if (Default.showLiveMinMax || 
                (info.AmperageMax > _maxAmperage || _maxAmperage == 0.0F))
                _maxAmperage = info.AmperageMax;

            amperageAvgBlock.Text = info.AmperageAvg.ToString("0.00") + " A";
            amperageMinBlock.Text = _minAmperage.ToString("0.00") + " A";
            amperageMaxBlock.Text = _maxAmperage.ToString("0.00") + " A";

            setVoltageColor(voltageAvgBlock, info.VoltageAvg);
            setVoltageColor(voltageMinBlock, _minVoltage);
            setVoltageColor(voltageMaxBlock, _maxVoltage);

            setAmperageColor(amperageAvgBlock, info.AmperageAvg);
            setAmperageColor(amperageMinBlock, _minAmperage);
            setAmperageColor(amperageMaxBlock, _maxAmperage);
        }

        private void SetDisplayColors(Brush color)
        {
            TextBlock[] textBlocks = new TextBlock[]
            {
                voltageAvgBlock,
                voltageMinBlock,
                voltageMaxBlock,
                amperageAvgBlock,
                amperageMaxBlock,
                amperageMinBlock
            };

            foreach (TextBlock textBlock in textBlocks)
                textBlock.Foreground = color;
        }

        private string FormatUptime(int seconds)
        {
            List<string> returnParts = new List<string>();

            if (seconds > 86400)
            {
                int days = seconds / 86400;
                returnParts.Add(days.ToString() + "d");
                seconds -= days * 86400;
            }

            if (seconds > 3600)
            {
                int hours = seconds / 3600;
                returnParts.Add(hours.ToString() + "h");
                seconds -= hours * 3600;
            }

            if (seconds > 60)
            {
                int minutes = seconds / 60;
                returnParts.Add(minutes.ToString() + "m");
                seconds -= minutes * 60;
            }

            if (seconds > 0)
            {
                returnParts.Add(seconds.ToString() + "s");
            }

            return String.Join(" ", returnParts);
        }

        private void setVoltageColor(TextBlock block, float Value)
        {
            if (Value < Default.warningVoltage)
                block.Foreground = (SolidColorBrush)(new BrushConverter().ConvertFrom(Default.warningColor));
            else if (Value >= Default.warningVoltage && Value < Default.watchVoltage)
                block.Foreground = (SolidColorBrush)(new BrushConverter().ConvertFrom(Default.watchColor));
            else
                block.Foreground = (SolidColorBrush)(new BrushConverter().ConvertFrom(Default.normalColor));
        }

        private void setAmperageColor(TextBlock block, float Value)
        {
            if (Value > Default.warningAmperage)
                block.Foreground = (SolidColorBrush)(new BrushConverter().ConvertFrom(Default.warningColor));
            else if (Value < Default.warningAmperage && Value >= Default.watchAmperage)
                block.Foreground = (SolidColorBrush)(new BrushConverter().ConvertFrom(Default.watchColor));
            else
                block.Foreground = (SolidColorBrush)(new BrushConverter().ConvertFrom(Default.normalColor));
        }

        private void controls_enable()
        {
            button_refreshPorts.IsEnabled = true;
            comPortList.IsEnabled = true;
        }

        private void controls_disable()
        {
            button_refreshPorts.IsEnabled = false;
            comPortList.IsEnabled = false;
        }

        private void statusBar_SetConnected()
        {
            connectionStatusBlock.Text = "Connected to " + _comPort;
        }

        private void statusBar_SetDisconnected()
        {
            connectionStatusBlock.Text = "Disconnected";
            uptimeStatusBlock.Text = "";
        }

        private void comPortList_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            button_Connect.IsEnabled = (comPortList.SelectedItem != null);
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (_connected)
                Disconnect();
        }

        private void button_logDirectoryBrowse_Click(object sender, RoutedEventArgs e)
        {
            CommonOpenFileDialog dialog = new CommonOpenFileDialog();
            dialog.IsFolderPicker = true;

            if (Directory.Exists(Default.logDirectory))
            {
                dialog.RestoreDirectory = true;
                dialog.InitialDirectory = Default.logDirectory;
            }

            CommonFileDialogResult result = dialog.ShowDialog();

            if (result == CommonFileDialogResult.Ok)
                Default.logDirectory = dialog.FileName;

            this.Focus();
        }

        private void resetSettingsButton_Click(object sender, RoutedEventArgs e)
        {
            MessageBoxResult result = MessageBox.Show(
                "Are you sure you want to reset settings to the defaults. This will also disconnect any current session.",
                "Confirm reset",
                MessageBoxButton.YesNo);

            if (result == MessageBoxResult.Yes)
            {
                if (_connected)
                    Disconnect();

                if (_logFile != null)
                    CloseLogFile();

                Default.Reset();

                SetDefaultLogDirectory();
            }
        }

        private void ReadEEPROM()
        {
            WriteToDevice("GET Config");
            deviceConfig.EEPROMRead = true;
        }

        private void WriteToDevice(string line)
        {
            if (_serialPort != null)
            {
                if (_serialPort.IsOpen)
                {
                    _serialPort.Write(line + "\r\n");
                    Thread.Sleep(250);
                }
            }
        }

        private void SaveEEPROM()
        {
            WriteToDevice("SET AverageReadingCount " + deviceConfig.AverageReadingCount.ToString());
            WriteToDevice("SET UpdateFrequency " + deviceConfig.UpdateFrequency.ToString());
            WriteToDevice("SET WriteInterval " + deviceConfig.WriteInterval.ToString());
            WriteToDevice("SET VoltageCalibration " + deviceConfig.VoltageCalibration.ToString("0.0000"));
            WriteToDevice("SET R1Actual " + deviceConfig.R1Actual.ToString());
            WriteToDevice("SET R2Actual " + deviceConfig.R2Actual.ToString());
            WriteToDevice("SET AmpDigitalOffset " + deviceConfig.AmpDigitalOffset.ToString());

            WriteToDevice("SAVE");
        }

        private void readEepromButton_Click(object sender, RoutedEventArgs e)
        {
            ReadEEPROM();
        }

        private void resetEepromButton_Click(object sender, RoutedEventArgs e)
        {
            WriteToDevice("CLEAR");

            ReadEEPROM();
        }

        private void PauseReadings()
        {
            WriteToDevice("PAUSE");
            _serialPort.ReadExisting();
        }

        private void ResumeReadings()
        {
            WriteToDevice("RESUME");
        }

        private void writeEepromButton_Click(object sender, RoutedEventArgs e)
        {
            SaveEEPROM();
        }
    }
}
