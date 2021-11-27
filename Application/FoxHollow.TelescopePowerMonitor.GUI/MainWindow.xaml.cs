using FoxHollow.TelescopePowerMonitor.DeviceClient;
using FoxHollow.TelescopePowerMonitor.DeviceClient.Exceptions;
using Microsoft.WindowsAPICodePack.Dialogs;
using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using static BatteryMonitorGUI.Properties.Settings;

namespace BatteryMonitorGUI
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public TpmClient tpmClient { get; }

        public MainWindow()
        { 
            InitializeComponent();
            tpmClient = new TpmClient();
            tpmClient.OnLogLine += AppendLog;
            tpmClient.OnPowerChanged += UpdatePowerInfo;
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            button_Connect.IsEnabled = false;

            SetDefaultLogDirectory();

            uptimeStatusBlock.Text = "---";
            mainWindow.Topmost = Default.alwaysOnTop;

            statusBar_SetDisconnected();
            refreshPorts();
            ClearStatusValues();

            setLastComPort();

            attachValueWatchers();
        }

        private void SetDefaultLogDirectory()
        {
            if (String.IsNullOrWhiteSpace(Default.logDirectory))
            {
                Default.logDirectory =
                    Path.Combine(
                        Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
                        "TelescopePowerMonitor"
                    );

                Default.Save();
            }
        }

        private void attachValueWatchers()
        {
            Default.PropertyChanged += Default_PropertyChanged1;
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
                    {
                        tpmClient.SetSerialPort(Default.lastComPort);
                        Connect();
                    }
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
            if (!tpmClient.Connected)
            {
                if (comPortList.SelectedItem == null)
                {
                    MessageBox.Show("You must select a COM port to connect to");
                    return;
                }


                tpmClient.SetSerialPort(comPortList.SelectedItem.ToString());
                Connect();
            }
            else
                Disconnect();
        }

        private void Connect()
        {
            if (tpmClient.Connected)
                return;

            try
            {
                tpmClient.Connect();

                if (tpmClient.Connected)
                {
                    statusBar_SetConnected();
                    controls_disable();
                    button_Connect.Content = "Disconnect";

                    Default.lastComPort = tpmClient.Port;
                    Default.Save();

                    uptimeStatusBlock.Foreground = Brushes.DarkGray;

                    SetDisplayColors(Brushes.DarkGray);
                }
            }
            catch (SerialPortNotFoundException e)
            {
                MessageBox.Show(e.Message);
                return;
            }
        }

        private void Disconnect()
        {
            if (!tpmClient.Connected)
                return;

            Task.Run(() => tpmClient.Disconnect());

            statusBar_SetDisconnected();
            controls_enable();
            button_Connect.Content = "Connect";

            ClearStatusValues();
        }

        private void WriteLogLine(string line)
        {
            // Create log directory
            if (!Directory.Exists(Default.logDirectory))
                Directory.CreateDirectory(Default.logDirectory);

            string logFileName = "BatteryLog_" + DateTime.UtcNow.ToString("yyyy-MM-dd") + ".txt";

            string logPath = Path.Combine(Default.logDirectory, logFileName);

            File.AppendAllText(logPath, line);
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
            
            // TODO: fix the scroll issue here when the TextBox tab isn't active
            serialLogBlock.AppendText(value + Environment.NewLine);
            serialLogBlock.ScrollToEnd();

            WriteLogLine(value);
        }

        public void ClearStatusValues()
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

        public void UpdatePowerInfo(PowerInfo info)
        {
            if (!Dispatcher.CheckAccess())
            {
                this.Dispatcher.Invoke(() => UpdatePowerInfo(info));

                return;
            }

            //uptimeStatusBlock.Text = FormatUptime(info.UptimeSeconds);

            voltageAvgBlock.Text = info.Voltage.Current.ToString("0.00") + " v";
            voltageMinBlock.Text = info.Voltage.Minimum.ToString("0.00") + " v";
            voltageMaxBlock.Text = info.Voltage.Maximum.ToString("0.00") + " v";

            amperageAvgBlock.Text = info.BatteryAmperage.Current.ToString("0.00") + " A";
            amperageMinBlock.Text = info.BatteryAmperage.Minimum.ToString("0.00") + " A";
            amperageMaxBlock.Text = info.BatteryAmperage.Maximum.ToString("0.00") + " A";

            setVoltageColor(voltageAvgBlock, info.Voltage.Current);
            setVoltageColor(voltageMinBlock, info.Voltage.Minimum);
            setVoltageColor(voltageMaxBlock, info.Voltage.Maximum);

            setAmperageColor(amperageAvgBlock, info.BatteryAmperage.Current);
            setAmperageColor(amperageMinBlock, info.BatteryAmperage.Minimum);
            setAmperageColor(amperageMaxBlock, info.BatteryAmperage.Maximum);
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
            connectionStatusBlock.Text = $"Connected to {tpmClient.Port}";
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
            if (tpmClient.Connected)
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
                this.Disconnect();
                Default.Reset();

                this.SetDefaultLogDirectory();
            }
        }

        private void readEepromButton_Click(object sender, RoutedEventArgs e)
            => tpmClient.ReadEEPROM();

        private void resetEepromButton_Click(object sender, RoutedEventArgs e)
            => tpmClient.ResetEeprom();

        private void writeEepromButton_Click(object sender, RoutedEventArgs e)
            => tpmClient.SaveEEPROM();
    }
}
