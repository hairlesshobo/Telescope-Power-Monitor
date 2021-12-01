﻿using FoxHollow.TelescopePowerMonitor.DeviceClient;
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
using static FoxHollow.TelescopePowerMonitor.GUI.Properties.Settings;

namespace FoxHollow.TelescopePowerMonitor.GUI
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public TpmClient TpmClient { get; }

        public MainWindow()
        { 
            this.TpmClient = new TpmClient();
            this.TpmClient.OnLogLine += AppendLog;

            App.TpmClient = this.TpmClient;

            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            button_Connect.IsEnabled = false;

            SetDefaultLogDirectory();

            mainWindow.Topmost = Default.alwaysOnTop;

            StatusBar_SetDisconnected();
            RefreshPorts();

            SetLastComPort();

            AttachValueWatchers();
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

        private void AttachValueWatchers()
        {
            Default.PropertyChanged += Default_PropertyChanged1;
        }

        private void Default_PropertyChanged1(object sender, System.ComponentModel.PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(Default.watchVoltage))
            {
                if (Default.watchVoltage == Default.minVoltageRange)
                    Default.warningVoltage = Default.minVoltageRange;
                else
                {
                    if (Default.warningVoltage >= Default.watchVoltage)
                        Default.warningVoltage = Default.watchVoltage - 0.1F;
                }
            }

            if (e.PropertyName == nameof(Default.warningVoltage))
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

        private void SetLastComPort()
        {
            if (Default.lastComPort != "")
            {
                if (((List<string>)comPortList.ItemsSource).Contains(Default.lastComPort))
                {
                    comPortList.SelectedItem = Default.lastComPort;

                    if (Default.autoConnectLast)
                    {
                        TpmClient.SetSerialPort(Default.lastComPort);
                        Connect();
                    }
                }
            }
        }

        private void RefreshPortsButton_Click(object sender, RoutedEventArgs e)
            => RefreshPorts();

        private void RefreshPorts()
        {
            comPortList.ItemsSource = new List<string>() { "Loading.." };

            List<string> comPorts = SerialPort.GetPortNames().ToList();

            comPortList.ItemsSource = comPorts;
        }

        private void ConnectButton_Click(object sender, RoutedEventArgs e)
        {
            if (!TpmClient.Connected)
            {
                if (comPortList.SelectedItem == null)
                {
                    MessageBox.Show("You must select a COM port to connect to");
                    return;
                }


                TpmClient.SetSerialPort(comPortList.SelectedItem.ToString());
                Connect();
            }
            else
                Disconnect();
        }

        private void Connect()
        {
            if (TpmClient.Connected)
                return;

            try
            {
                TpmClient.Connect();

                if (TpmClient.Connected)
                {
                    StatusBar_SetConnected();
                    Controls_disable();
                    button_Connect.Content = "Disconnect";

                    Default.lastComPort = TpmClient.Port;
                    Default.Save();
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
            if (!TpmClient.Connected)
                return;

            Task.Run(() => TpmClient.Disconnect());

            StatusBar_SetDisconnected();
            Controls_enable();
            button_Connect.Content = "Connect";
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

        private void Controls_enable()
        {
            button_refreshPorts.IsEnabled = true;
            comPortList.IsEnabled = true;
        }

        private void Controls_disable()
        {
            button_refreshPorts.IsEnabled = false;
            comPortList.IsEnabled = false;
        }

        private void StatusBar_SetConnected()
        {
            connectionStatusBlock.Text = $"Connected to {TpmClient.Port}";
        }

        private void StatusBar_SetDisconnected()
        {
            connectionStatusBlock.Text = "Disconnected";
        }

        private void ComPortList_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            button_Connect.IsEnabled = (comPortList.SelectedItem != null);
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (TpmClient.Connected)
                Disconnect();
        }

        private void Button_logDirectoryBrowse_Click(object sender, RoutedEventArgs e)
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

        private void ResetSettingsButton_Click(object sender, RoutedEventArgs e)
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

        private void ReadEepromButton_Click(object sender, RoutedEventArgs e)
            => TpmClient.ReadEEPROM();

        private void ResetEepromButton_Click(object sender, RoutedEventArgs e)
            => TpmClient.ResetEeprom();

        private void WriteEepromButton_Click(object sender, RoutedEventArgs e)
            => TpmClient.SaveEEPROM();

        private void timeDeltaStatusItem_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            TpmClient.UpdateTime();
        }

        private void ToggleDehum_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
            => this.TpmClient.AutoDehumToggleState();

        private void ToggleScope_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
            => this.TpmClient.TelescopeToggleState();

        private void ToggleAux1_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
            => this.TpmClient.Aux1ToggleState();

        private void ToggleDehumidifier_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            //this.TpmClient.DehumidifierToggleState();
        }

        private void ToggleAc_MouseDoubleClick(object sender, System.Windows.Input.MouseButtonEventArgs e)
            => this.TpmClient.AcToggleState();
    }
}
