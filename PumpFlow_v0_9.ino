/*
      PumpFlow
      Used to calculate pump flow rate from an rpm sensor, pressure meter, and pump curves.
  
      Copyright (C) 2017  Duncan Thomson  <thomson.duncan@gmail.com>

      This program is free software: you can redistribute it and/or modify
      it under the terms of the GNU General Public License as published by
      the Free Software Foundation, either version 3 of the License, or
      (at your option) any later version.

      This program is distributed in the hope that it will be useful,
      but WITHOUT ANY WARRANTY; without even the implied warranty of
      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
      GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
      along with this program.  If not, see <http://www.gnu.org/licenses/>.
      
      =========================================================================
      Pins:
        D02 - rpm sensor
        D05 - pwm output to LED display (upper line)
        D06 - pwm output to LED display (lower line)
        D10 - Serial TX to RS232 converter
        D11 - Serial RX to RS232 converter
        A04 - I2C pressure reading (SDA)
        A05 - I2C pressure reading (SCL)
      =========================================================================
*/

#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <SoftwareSerial.h> 

Adafruit_ADS1115 ads1115; // ADC instance
SoftwareSerial rs232(11, 10); // RX, TX

/*
  Pump curve data
    Polynomial coefficients
    x3, x2, x1, c
*/
const float pumpCurves[7][4] = {
{0, 0, 0, 0}, // 0 rpm
{-3.9771940752, 13.9272296157, -21.04655348, 44.5164453674}, // 1000 rpm
{-0.051048951, 0.7024475524, -6.9601398601, 87.0954545455}, // 2000 rpm
{-0.0121905463, 0.3935141, -6.3307213799, 134.8056618329}, // 3000 rpm
{-0.0040141114, 0.2402386873, -6.067474348, 184.2294141506}, // 4000rpm
{-0.0010699358, 0.09835431, -4.1976130146, 227.0055724173}, // 5000 rpm
{-0.0003262808, 0.0412112425, -2.8293631878, 265.0688963211} // 6000 rpm
};

//Pins for analog outputs going to LED display
const int pinLedUpper = 5;
const int pinLedLower = 6;

const int pinRpm = 2;  // pin number for RPM sensor - must be usable by interrupt (pin 2 or 3)
const int pulsePerRev = 2;  //Pulses per revolution
volatile int pulseCount = 0;  // interrupt vars must be volatile
volatile unsigned long pulseTimePrev = 0;
volatile unsigned long pulseTime = 0;

unsigned long currentMillis;
unsigned long task1PrevMillis = 0;
const long task1Interval = 500;  //Sampling interval


/*
  Setup routine
*/
void setup(void)
{
  // Initialise USB port
  Serial.begin(9600);
  Serial.println("PUMPFLOW - Copyright (C) 2017  Duncan Thomson  <thomson.duncan@gmail.com>");
  Serial.println("Licensed under GNU General Public License v3.0");
  Serial.println;
  Serial.println("USB serial output"); 
  
  // Initialise RS232 port
  rs232.begin(9600);
  rs232.println("PUMPFLOW - Copyright (C) 2017  Duncan Thomson  <thomson.duncan@gmail.com>");
  rs232.println("Licensed under GNU General Public License v3.0");
  rs232.println;
  rs232.println("RS232 serial output");
  
  // Initialise ADC board
  ads1115.begin();

  // Initialise RPM pin
  pinMode (pinRpm, INPUT);
  digitalWrite (pinRpm, HIGH); //enable internal 20k pulllup resistor
  attachInterrupt(digitalPinToInterrupt(pinRpm), encoderInterrupt, RISING); // RISING, HIGH
  
}
 
/*
  Main loop
*/
void loop(void)
{
  //unsigned long  
  currentMillis = millis(); // millisecond count for scheduled tasks
  String strSerial = "";
  
  //int16_t adc0;
  int ledUpper, ledLower;
 
  // Task 1
  unsigned long task1ElapsedTime = currentMillis - task1PrevMillis;
  
  if(task1ElapsedTime >= task1Interval){
    detachInterrupt(digitalPinToInterrupt(pinRpm));  //pause pulse counting

    // Calculate RPM
    float rpm = calcRpm(pulseCount, pulsePerRev, task1Interval);
    strSerial += "R:" + String(rpm, 1);

    // Read pressure
    float pressMbar = calcPressure();
    float pressMetres = calcPressMetres(pressMbar);
    float pressPsi = calcPressPsi(pressMbar);
    float pressBar = calcPressBar(pressMbar);
    strSerial += ",P:" + String(pressBar, 2);
    
    //Calculate flow
    float flow = calcFlow(rpm, pressMetres); //(5500.0, 40.0)
    strSerial += ",F:" + String(flow, 3);

    Serial.println(strSerial);
    rs232.println(strSerial);

/*    // Display rpm
    ledUpper = map(rpm, 0, 6000, 0, 265);
    analogWrite(pinLedUpper, ledUpper);
*/
    // Display pressure
    ledUpper = map(pressMbar, -12000, 12000, 0, 255);// Scaled for mBar
    analogWrite(pinLedUpper, ledUpper);

    // Display flow
    ledLower = map(flow, 0, 250, 0, 255);
    analogWrite(pinLedLower, ledLower);
    
    //clear rpm ready for new samples
    pulseCount = 0;
    task1PrevMillis = millis();

    //Re-enable interrupt
    attachInterrupt(digitalPinToInterrupt(pinRpm), encoderInterrupt, RISING); // RISING, HIGH
  }
}

float calcRpm (int pulseCount, int pulsePerRev, int task1Interval){
  float rpm;
  rpm = (60.0/pulsePerRev) * pulseCount * (1000.0/task1Interval);
  return rpm;  
}

float calcPressure(){
  int16_t adc0;
  float  pressMbar, pressPsi, pressBar, pressMetres;
  adc0 = ads1115.readADC_SingleEnded(0);
  pressMbar = map(adc0, 5101, 25532, -12000, 12000); // pressure in millibar
  //pressBar = pressMbar / 1000.0;
  //pressPsi = pressMbar * 0.0145038;
  //pressMetres = pressMbar * 0.01019977334;
  return pressMbar;
}

float calcPressMetres(float pressMbar){
  return pressMbar * 0.01019977334;
}

float calcPressPsi(float pressMbar){
  return pressMbar * 0.0145038;
}

float calcPressBar(float pressMbar){
  return pressMbar / 1000.0;
}

float calcFlow(float rpm, float pressure){
  float xx3, xx2, xx1, x3, x2, x1, c, flow;
  float xval;
  
  float flowC1, flowC2;
  int curve1, curve2;
  
  float fracRpm;
  
  if (rpm < 1000){
    curve1 = 0; // 0 rpm curve
    curve2 = 1; // 1000 rpm curve
    fracRpm = rpm / 1000.0;
  }else if (rpm < 2000){
    curve1 = 1; // 1000 rpm curve
    curve2 = 2; // 2000 rpm curve
    fracRpm = (rpm - 1000.0)/1000.0;
  }else if(rpm < 3000){
    curve1 = 2; // 2000 rpm curve
    curve2 = 3; // 3000 rpm curve
    fracRpm = (rpm - 2000.0)/1000.0;
  }else if (rpm < 4000){
    curve1 = 3; // 3000 rpm curve
    curve2 = 4; // 4000 rpm curve
    fracRpm = (rpm - 3000.0)/1000.0;
  }else if (rpm < 5000){
    curve1 = 4; // 4000 rpm curve
    curve2 = 5; // 5000 rpm curve
    fracRpm = (rpm - 4000.0)/1000.0;
  }else if (rpm < 6000){
    curve1 = 5; // 5000 rpm curve
    curve2 = 6; // 6000 rpm curve
    fracRpm = (rpm - 5000.0)/1000.0;
  }else if (rpm >= 6000){
    curve1 = 6; // 6000 rpm curve
    curve2 = 6; // 6000 rpm curve
    fracRpm = 0.0;
  }

  pressure = abs(pressure); // Pressure must be positive to register on pump curve

  flowC1 = curveLookup(curve1, pressure);
  flowC2 = curveLookup(curve2, pressure);
  
  flow = flowC1 + (flowC2-flowC1) * fracRpm;
  flow = flow * 0.85;
  return flow;
}

float curveLookup(int curve, float pressure){
  float xx3, xx2, xx1, x3, x2, x1, c, flow;
  float xval;
  
  x3 = pumpCurves[curve][0];
  x2 = pumpCurves[curve][1];
  x1 = pumpCurves[curve][2];
  c = pumpCurves[curve][3];
  
  xval = pressure;
  xx3 = x3*(xval*xval*xval);
  xx2 = x2*(xval*xval);
  xx1 = x1*(xval);
  flow = xx3 + xx2 + xx1 + c;  
  
  //flow cannot be less than 0
  if (flow < 0.0){
   flow = 0.0;
  } 
 
  /*
  Serial.print(xval); Serial.println();
  Serial.print(xx3); Serial.println();
  Serial.print(xx2); Serial.println();
  Serial.print(xx1); Serial.println();
  Serial.print(c); Serial.println();
  Serial.print(flow); Serial.println();
  */
  
  return flow;  
}

void encoderInterrupt()
{
  pulseCount++;
  pulseTimePrev = pulseTime;
  pulseTime = millis();
}
