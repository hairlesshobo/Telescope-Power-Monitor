using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Data;

namespace FoxHollow.TelescopePowerMonitor.GUI.Converters
{
    class UptimeConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
            => (App.TpmClient.Connected ? Formatting.FormatUptime((int)value) : Formatting.Empty);

        public object ConvertBack(object value, Type targetTypes, object parameter, CultureInfo culture)
            => throw new NotSupportedException();
    }
}
