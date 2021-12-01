using System;
using System.Collections.Generic;
using System.Text;

namespace FoxHollow.TelescopePowerMonitor.DeviceClient
{
    internal interface IDeviceLogType
    {
        DateTimeOffset LastReadDtm { get; }
        int LastReadUptime { get; }
    }
}
