using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace TrackingHand
{
    class MainWindowViewModel
    {
        // Sensor elevation angle
        private int sensorAngleValue;

        // Gets or sets the sensor angle.
        public int SensorAngle
        {
            get
            {
                return this.sensorAngleValue;
            }

            set
            {
                if (this.sensorAngleValue != value)
                {
                    this.sensorAngleValue = value;
                }
            }
        }
    }
}
