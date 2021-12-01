using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Data;

namespace FoxHollow.TelescopePowerMonitor.GUI.Converters
{
    class SwitchBackgroundConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (App.TpmClient == null || !App.TpmClient.Connected)
                return "#FF000000";

            return (bool)value ? "#FF008000" : "#FFF10000";
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
            => new NotImplementedException();
    }
}
