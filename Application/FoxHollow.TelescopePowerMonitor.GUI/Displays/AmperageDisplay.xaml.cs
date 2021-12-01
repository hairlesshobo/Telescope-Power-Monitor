using FoxHollow.TelescopePowerMonitor.DeviceClient;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using static FoxHollow.TelescopePowerMonitor.GUI.Properties.Settings;

namespace FoxHollow.TelescopePowerMonitor.GUI
{
    /// <summary>
    /// Interaction logic for VoltageDisplay.xaml
    /// </summary>
    public partial class AmperageDisplay : UserControl
    {
        public string Title
        {
            get { return GetValue(TitleProperty) as string; }
            set { SetValue(TitleProperty, value); }
        }

        public bool IsActive
        {
            get { return (bool)GetValue(IsActiveProperty); }
            set { SetValue(IsActiveProperty, value); }
        }

        public FloatValue Value
        {
            get { return GetValue(ValueProperty) as FloatValue; }
            set { SetValue(ValueProperty, value); }
        }

        public AmperageDisplay()
        {
            InitializeComponent();

            ResetView();
        }

        private void ValueChanged(object sender, PropertyChangedEventArgs e)
        {
            if (!Dispatcher.CheckAccess())
            {
                this.Dispatcher.Invoke(() => ValueChanged(sender, e));

                return;
            }

            if (this.Value.Valid && this.IsActive)
            {
                this.Current.Text = Formatting.FormatAmperage(this.Value.Current);
                this.Minimum.Text = Formatting.FormatAmperage(this.Value.Minimum);
                this.Maximum.Text = Formatting.FormatAmperage(this.Value.Maximum);

                SetAmperageColor(this.Current, this.Value.Current);
                SetAmperageColor(this.Minimum, this.Value.Minimum);
                SetAmperageColor(this.Maximum, this.Value.Maximum);
            }
            else
                ResetView();
        }

        private void ResetView()
        {
            this.Current.Text = Formatting.Empty;
            this.Minimum.Text = Formatting.Empty;
            this.Maximum.Text = Formatting.Empty;

            this.Current.Foreground = Brushes.Gray;
            this.Minimum.Foreground = Brushes.Gray;
            this.Maximum.Foreground = Brushes.Gray;
        }

        private static void SetAmperageColor(TextBlock block, float Value)
        {
            if (Value > Default.warningAmperage)
                block.Foreground = (SolidColorBrush)(new BrushConverter().ConvertFrom(Default.warningColor));
            else if (Value < Default.warningAmperage && Value >= Default.watchAmperage)
                block.Foreground = (SolidColorBrush)(new BrushConverter().ConvertFrom(Default.watchColor));
            else
                block.Foreground = (SolidColorBrush)(new BrushConverter().ConvertFrom(Default.normalColor));
        }

        private static void OnValuePropertyChanged(DependencyObject sender, DependencyPropertyChangedEventArgs e)
        {
            if (e.Property.Name == nameof(Value))
                (e.NewValue as FloatValue).PropertyChanged += ((AmperageDisplay)sender).ValueChanged;

            if (e.Property.Name == nameof(IsActive) && ((bool)e.NewValue) == false)
                ((AmperageDisplay)sender).ResetView();
        }

        public static readonly DependencyProperty ValueProperty = DependencyProperty.Register("Value", typeof(FloatValue), typeof(AmperageDisplay), new UIPropertyMetadata(new FloatValue(), OnValuePropertyChanged));
        public static readonly DependencyProperty IsActiveProperty = DependencyProperty.Register("IsActive", typeof(bool), typeof(AmperageDisplay), new UIPropertyMetadata(false, OnValuePropertyChanged));
        public static readonly DependencyProperty TitleProperty = DependencyProperty.Register("Title", typeof(string), typeof(AmperageDisplay), new UIPropertyMetadata("", OnValuePropertyChanged));
    }
}
