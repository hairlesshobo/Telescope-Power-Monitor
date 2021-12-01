using FoxHollow.TelescopePowerMonitor.DeviceClient;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using static FoxHollow.TelescopePowerMonitor.GUI.Properties.Settings;

namespace FoxHollow.TelescopePowerMonitor.GUI
{
    /// <summary>
    /// Interaction logic for SwitchDisplay.xaml
    /// </summary>
    public partial class SwitchDisplay : UserControl //, INotifyPropertyChanged
    {
        public string Title
        {
            get { return GetValue(TitleProperty) as string; }
            set { SetValue(TitleProperty, value); }
        }

        public bool State
        {
            get { return (bool)GetValue(StateProperty); }
            set { SetValue(StateProperty, value); }
        }

        public SwitchDisplay()
        {
            InitializeComponent();
        }

        private void StateChanged(object sender, PropertyChangedEventArgs e)
        {
            if (!Dispatcher.CheckAccess())
            {
                this.Dispatcher.Invoke(() => StateChanged(sender, e));

                return;
            }
        }

        private static void OnValuePropertyChanged(DependencyObject sender, DependencyPropertyChangedEventArgs e)
        {
            if (e.Property.Name == nameof(State)) // && ((bool)e.NewValue) == false)
            {
                // do something here if the state changed
            }
        }

        public static readonly DependencyProperty StateProperty = DependencyProperty.Register("State", typeof(bool), typeof(SwitchDisplay), new UIPropertyMetadata(false, OnValuePropertyChanged));
        public static readonly DependencyProperty TitleProperty = DependencyProperty.Register("Title", typeof(string), typeof(SwitchDisplay), new UIPropertyMetadata(" ", OnValuePropertyChanged));

        //public event PropertyChangedEventHandler PropertyChanged;

        //protected void OnPropertyChanged([CallerMemberName] string name = null)
        //    => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
    }
}
