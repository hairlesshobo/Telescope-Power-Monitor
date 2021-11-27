using System;
using System.Collections.Generic;
using System.Text;

namespace FoxHollow.TelescopePowerMonitor.DeviceClient.Exceptions
{
    public class SerialPortNotFoundException : Exception
    {
        public SerialPortNotFoundException(string port) : base($"Serial port {port} does not exist!")
        { }
    }
}
