// ----------------------------------------------------------------------------
// Barometric Altimeter Demo Code

// A simple code that reads sensor data from the BMP388 pressure/temperature
// sensor and computes a change in altitude via the Hypsometric equation

// Code By: Michael Wrona | GitHub: michaelwro | YouTube: MicWro Engr
// Created: 21 July 2020
// ----------------------------------------------------------------------------
/**
 * Pin Configuration (I2C)
 * -----------------
 * VIN - Arduino 3.3V supply
 * GND - Arduino ground
 * SCK - Arduino A5
 * SDI - Arduino A4
 * 
 * Legal
 * -----
 * I, Michael Wrona, do not take credit for the BMP388 sensor library. 
 * The BMP3XX sensor library was developed by Adafruit Industries. The
 * link to this library can be found in the Resources section in this
 * document.
 * 
 * Resources
 * ---------
 * ~ Adafruit BMP388 Sensor Library (GitHub):
 *     https://github.com/adafruit/Adafruit_BMP3XX
 * ~ Adafruit BMP388 Sensor (Adafruit Industries):
 *     https://www.adafruit.com/product/3966
 * ~ Hypsometric Equation (Wikipedia):
 *     https://en.wikipedia.org/wiki/Hypsometric_equation
 * ~ Virtual Temperature Calculator (NOAA):
 *     https://www.weather.gov/epz/wxcalc_virtualtemperature
 */


// Include libraries
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"


// Function templates
void DetermineGroundPressure(uint8_t n);
float HypsometricEquation(float currentPres);


// Create classes
Adafruit_BMP3XX altimeter; // I2C init.


// Global variables
float virtualTemp = 305.0f;  // [K]
float altitude;  // [m]
float groundPressure;  // [Pa]


void setup() {
    // Init. serial port
    Serial.begin(115200);

    // Init. sensor
    if (altimeter.begin() == false) {
        Serial.println("Could not find a valid BMP3 sensor, check wiring!");
        while (1);
    }

    // Config. oversampling and filtering
    Serial.println("Initializing sensor...");
    altimeter.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    altimeter.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    altimeter.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

    // Determine ground-level pressure
    DetermineGroundPressure(15);
}


void loop() {
    // Perform reading
    if (altimeter.performReading() == false) {
        Serial.println("Sensor read failed!");
        return;
    }


    // Compute change in altitude
    altitude = HypsometricEquation(altimeter.pressure);  // [m]


    // Print results!
    Serial.println("-----------------------");
    Serial.print("Temperature (C): ");
    Serial.println(altimeter.temperature, 6);

    Serial.print("Pressure (hPa/mb): ");
    Serial.println(altimeter.pressure / 100.0, 6);

    Serial.print("Altitude Change (m): ");
    Serial.println(altitude, 6);

    delay(200);  // Fs = 5Hz
}



/**
 * Read sensor, determine average pressure at ground-level
 * 
 * @param n  Number of readings to average
 */
void DetermineGroundPressure(uint8_t n) {
    uint8_t i;

    // Perform a few readings to 'flush' bad data from the sensor
    for (i = 0; i < 3; i++) {
        if (altimeter.performReading() == false) {
            Serial.println("Sensor read failed!");
            return;
        }
        delay(100);
    }

    // Perform 'n' readings and determine the average pressure
    Serial.println("Computing ground-level pressure...");
    groundPressure = 0.0f;
    for (i = 0; i < n; i++) {
        if (altimeter.performReading() == false) {
            Serial.println("Sensor read failed!");
            return;
        }
        groundPressure += altimeter.pressure;  // [Pa]
        delay(100);
    }
    groundPressure /= (float)n;  // Average, [Pa]
}


/**
 * Use the Hypsometric equation to compute a change in altitude [m] from
 * a change in pressure [Pa].
 * 
 * @param currentPres  Current pressure reading [Pa]
 */
float HypsometricEquation(float currentPres) {
    float R = 287.0f;  // Gas const. [J / kg.K]
    float g = 9.81f;  // Grav. accel. [m/s/s]
    return ((R * virtualTemp) / g) * log(groundPressure / currentPres);  // [m]
}



