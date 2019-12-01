using GHIElectronics.TinyCLR.Pins;
using System;
using System.Collections;
using System.Diagnostics;
using System.Text;
using System.Threading;

namespace CompassNamespace
{
    class Program
    {
        static void Main()
        {
            var compass = new Compass(FEZ.GpioPin.D4);
            compass.StartTakingMeasurements();
            compass.MeasurementComplete += (sender, e) => { Debug.WriteLine($"Info : {e.ToString()}"); };
            Thread.Sleep(Timeout.Infinite);
        }
    }
}
