using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Data;

namespace FoxHollow.TelescopePowerMonitor.GUI.Converters
{
    class SwitchColorConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            string parameterStr = (string)parameter ?? "background";

            if (parameterStr == "background")
            {
                if (App.TpmClient == null || !App.TpmClient.Connected)
                    return "#FF000000";
                else
                    return (bool)value ? "#FF008000" : "#FFF10000";
            }
            else
            {
                if (value == null || !((bool)value))
                    return "#FF808080";
                else
                    return "#FF000000";
            }
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
            => new NotImplementedException();
    }
}
