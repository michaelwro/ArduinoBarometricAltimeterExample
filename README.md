# Arduino Barometric Altimeter Example

A simple example of how to use Adafruit's BMP388 temperature/pressure as a barometric altimeter.

**Code By:** Michael Wrona | YouTube: MicWro Engr

## Description

This Arduino code uses [Adafruit's BMP388 temperature/pressure](https://www.adafruit.com/product/3966) sensor to create a barometric altimeter. This code demonstrates using the [Hypsometric Equation](https://en.wikipedia.org/wiki/Hypsometric_equation) to convert changes in atmospheric presure to changes in altitude. The code can be easily modified to accomodate other sensors by changing how data is pulled from the sensor.

First, the code measures and stores the average atmospheric pressure at ground-level. Then, the Hypsometric equation is used to convert subsequent pressure measurements into changes in altitude. The virtual temperature constant in the code can be modified to suit the user's ambient operating enviroment.

## Dependencies

In order to use this code as-is, you must have Adafruit's [BMP3XX Sensor Library](https://github.com/adafruit/Adafruit_BMP3XX) and [Unified Sensor Driver](https://github.com/adafruit/Adafruit_Sensor) installed in your Arduino libraries folder.


## Resources

* [Virtual Temperature Calculator (NOAA)](https://www.weather.gov/epz/wxcalc_virtualtemperature)
