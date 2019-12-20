// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

/*
* DDS - Direct digital synthesis example for sinewave generation.
*
* The implementation of the DDS relies upon integer
* arithmetic. The size of accumulator is N-bits.
* Assuming that the period of the output signal is
* 2*pi rad, the maximum phase is represented by 2^N.
* During one sample period the phase increases by the
* phase increment which lead us to:
* - outputFrequency(phaseIncr) = (samplingFrequency / 2^N) * phaseIncr  (1)
*
* The later can be rewritten:
* - phaseIncr = outputFrequency * resolution
*
* where resolution is:
* - resolution = 2^N / samplingFrequency  (2)
*
* Also:
* - N=log2(samplingFrequency / stepFrequency)+0.5 (3)
* - maximumOutputFrequency = samplingFrequency / 2
*
* For a better analog reconstruction of the signal we use:
* - maximumOutputFrequency=(samplingFrequency / 4) (4)
*
* According to (4) and for a maximum output frequency of
* 15KHz, a 62.5KHz sampling frequency was chosen.
* with (3) we can derive the accumulator as 16 bits wide
* (for sake of commodity), and according to (2) we'll
* have a resolution of 1.0486. Since we're using fixed
* point arithmetic, we'll scale the resolution by a
* factor of 2^16 (to maintain some accuracy):
* - N=16, resolution=(2^16 * maximumOutputFrequency)  /  (samplingFrequency)
*
* The minimum frequency generated is for phaseIncr=1 :
* - minimumFrequency = samplingFrequency/2^N
*/

#include <avr/pgmspace.h>
#include <avr/sleep.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */


/*
// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



/*#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;*/

// Sinewave lookup table for waveform generation
static const uint8_t  sineTable100[] PROGMEM =
{128,131,134,137,140,143,146,149,
152,155,158,162,165,167,170,173,
176,179,182,185,188,190,193,196,
198,201,203,206,208,211,213,215,
218,220,222,224,226,228,230,232,
234,235,237,238,240,241,243,244,
245,246,248,249,250,250,251,252,
253,253,254,254,254,255,255,255,
255,255,255,255,254,254,254,253,
253,252,251,250,250,249,248,246,
245,244,243,241,240,238,237,235,
234,232,230,228,226,224,222,220,
218,215,213,211,208,206,203,201,
198,196,193,190,188,185,182,179,
176,173,170,167,165,162,158,155,
152,149,146,143,140,137,134,131,
128,124,121,118,115,112,109,106,
103,100,97,93,90,88,85,82,
79,76,73,70,67,65,62,59,
57,54,52,49,47,44,42,40,
37,35,33,31,29,27,25,23,
21,20,18,17,15,14,12,11,
10,9,7,6,5,5,4,3,
2,2,1,1,1,0,0,0,
0,0,0,0,1,1,1,2,
2,3,4,5,5,6,7,9,
10,11,12,14,15,17,18,20,
21,23,25,27,29,31,33,35,
37,40,42,44,47,49,52,54,
57,59,62,65,67,70,73,76,
79,82,85,88,90,93,97,100,
103,106,109,112,115,118,121,124};

static const uint8_t  sineTable090[] PROGMEM =
{115,117,120,123,126,128,131,134,
136,139,142,145,148,150,153,155,
158,161,163,166,169,171,173,176,
178,180,182,185,187,189,191,193,
196,198,199,201,203,205,207,208,
210,211,213,214,216,216,218,219,
220,221,223,224,225,225,225,226,
227,227,228,228,228,229,229,229,
229,229,229,229,228,228,228,227,
227,226,225,225,225,224,223,221,
220,219,218,216,216,214,213,211,
210,208,207,205,203,201,199,198,
196,193,191,189,187,185,182,180,
178,176,173,171,169,166,163,161,
158,155,153,150,148,145,142,139,
136,134,131,128,126,123,120,117,
115,111,108,106,103,100,98,95,
92,90,87,83,81,79,76,73,
71,68,65,63,60,58,55,53,
51,48,46,44,42,39,37,36,
33,31,29,27,26,24,22,20,
18,18,16,15,13,12,10,9,
9,8,6,5,4,4,3,2,
1,1,0,0,0,0,0,0,
0,0,0,0,0,0,0,1,
1,2,3,4,4,5,6,8,
9,9,10,12,13,15,16,18,
18,20,22,24,26,27,29,31,
33,36,37,39,42,44,46,48,
51,53,55,58,60,63,65,68,
71,73,76,79,81,83,87,90,
92,95,98,100,103,106,108,111};

static const uint8_t  sineTable065[] PROGMEM =
{83,85,87,89,91,92,94,96,
98,100,102,105,107,108,110,112,
114,116,118,120,122,123,125,127,
128,130,131,133,135,137,138,139,
141,143,144,145,146,148,149,150,
152,152,154,154,156,156,157,158,
159,159,161,161,162,162,163,163,
164,164,165,165,165,165,165,165,
165,165,165,165,165,165,165,164,
164,163,163,162,162,161,161,159,
159,158,157,156,156,154,154,152,
152,150,149,148,146,145,144,143,
141,139,138,137,135,133,131,130,
128,127,125,123,122,120,118,116,
114,112,110,108,107,105,102,100,
98,96,94,92,91,89,87,85,
83,80,78,76,74,72,70,68,
66,65,63,60,58,57,55,53,
51,49,47,45,43,42,40,38,
37,35,33,31,30,28,27,26,
24,22,21,20,18,17,16,14,
13,13,11,11,9,9,7,7,
6,5,4,3,3,3,2,1,
1,1,0,0,0,0,0,0,
0,0,0,0,0,0,0,1,
1,1,2,3,3,3,4,5,
6,7,7,9,9,11,11,13,
13,14,16,17,18,20,21,22,
24,26,27,28,30,31,33,35,
37,38,40,42,43,45,47,49,
51,53,55,57,58,60,63,65,
66,68,70,72,74,76,78,80};

static const uint8_t  sineTable045[] PROGMEM =
{57,58,60,61,63,64,65,67,
68,69,71,72,74,75,76,77,
79,80,81,83,84,85,86,88,
89,90,91,92,93,94,95,96,
98,99,99,100,101,102,103,104,
105,105,106,107,108,108,109,109,
110,110,111,112,112,112,112,113,
113,113,114,114,114,114,114,114,
114,114,114,114,114,114,114,113,
113,113,112,112,112,112,111,110,
110,109,109,108,108,107,106,105,
105,104,103,102,101,100,99,99,
98,96,95,94,93,92,91,90,
89,88,86,85,84,83,81,80,
79,77,76,75,74,72,71,69,
68,67,65,64,63,61,60,58,
57,55,54,53,51,50,49,47,
46,45,43,41,40,39,38,36,
35,34,32,31,30,29,27,26,
25,24,23,22,21,19,18,18,
16,15,14,13,13,12,11,10,
9,9,8,7,6,6,5,4,
4,4,3,2,2,2,1,1,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,1,1,2,2,2,3,4,
4,4,5,6,6,7,8,9,
9,10,11,12,13,13,14,15,
16,18,18,19,21,22,23,24,
25,26,27,29,30,31,32,34,
35,36,38,39,40,41,43,45,
46,47,49,50,51,53,54,55};

static const uint8_t  sineTable010[] PROGMEM =
{12,13,13,13,14,14,14,14,
15,15,15,16,16,16,17,17,
17,17,18,18,18,19,19,19,
19,20,20,20,20,21,21,21,
21,22,22,22,22,22,23,23,
23,23,23,23,24,24,24,24,
24,24,24,24,25,25,25,25,
25,25,25,25,25,25,25,25,
25,25,25,25,25,25,25,25,
25,25,25,25,25,24,24,24,
24,24,24,24,24,23,23,23,
23,23,23,22,22,22,22,22,
21,21,21,21,20,20,20,20,
19,19,19,19,18,18,18,17,
17,17,17,16,16,16,15,15,
15,14,14,14,14,13,13,13,
12,12,12,11,11,11,10,10,
10,10,9,9,9,8,8,8,
7,7,7,7,6,6,6,5,
5,5,5,4,4,4,4,4,
3,3,3,3,2,2,2,2,
2,2,1,1,1,1,1,1,
1,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
1,1,1,1,1,1,1,2,
2,2,2,2,2,3,3,3,
3,4,4,4,4,4,5,5,
5,5,6,6,6,7,7,7,
7,8,8,8,9,9,9,10,
10,10,10,11,11,11,12,12};

static const uint8_t  sawTable[] PROGMEM = {
0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,
0x9,0xa,0xb,0xc,0xd,0xe,0xf,0x10,
0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,
0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,0x30,
0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,
0x29,0x2a,0x2b,0x2c,0x2d,0x2e,0x2f,0x30,
0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,
0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,0x30,
0x30,0x41,0x42,0x43,0x44,0x45,0x46,0x47,
0x48,0x49,0x4a,0x4b,0x4c,0x4d,0x4e,0x4f,
0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,
0x58,0x59,0x5a,0x5b,0x5c,0x5d,0x5e,0x5f,
0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,
0x68,0x69,0x6a,0x6b,0x6c,0x6d,0x6e,0x6f,
0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,
0x78,0x79,0x7a,0x7b,0x7c,0x7d,0x7e,0x7f,
0x7e,0x7d,0x7c,0x7b,0x7a,0x79,0x78,0x77,
0x76,0x75,0x74,0x73,0x72,0x71,0x70,0x6f,
0x6e,0x6d,0x6c,0x6b,0x6a,0x69,0x68,0x67,
0x66,0x65,0x64,0x63,0x62,0x61,0x60,0x5f,
0x5e,0x5d,0x5c,0x5b,0x5a,0x59,0x58,0x57,
0x56,0x55,0x54,0x53,0x52,0x51,0x50,0x4f,
0x4e,0x4d,0x4c,0x4b,0x4a,0x49,0x48,0x47,
0x46,0x45,0x44,0x43,0x42,0x41,0x30,0x30,
0x3f,0x3e,0x3d,0x3c,0x3b,0x3a,0x39,0x38,
0x37,0x36,0x35,0x34,0x33,0x32,0x31,0x30,
0x2f,0x2e,0x2d,0x2c,0x2b,0x2a,0x29,0x28,
0x27,0x26,0x25,0x24,0x23,0x22,0x21,0x30,
0x1f,0x1e,0x1d,0x1c,0x1b,0x1a,0x19,0x18,
0x17,0x16,0x15,0x14,0x13,0x12,0x11,0x10,
0xf,0xe,0xd,0xc,0xb,0xa,0x9,0x8,
0x7,0x6,0x5,0x4,0x3,0x2,0x1,0x0};

static const uint8_t  squareTable006[] PROGMEM = {
15,15,15,15,15,15,15,15,
15,15,15,15,15,15,15,15,
15,15,15,15,15,15,15,15,
15,15,15,15,15,15,15,15,
15,15,15,15,15,15,15,15,
15,15,15,15,15,15,15,15,
15,15,15,15,15,15,15,15,
15,15,15,15,15,15,15,15,
15,15,15,15,15,15,15,15,
15,15,15,15,15,15,15,15,
15,15,15,15,15,15,15,15,
15,15,15,15,15,15,15,15,
15,15,15,15,15,15,15,15,
15,15,15,15,15,15,15,15,
15,15,15,15,15,15,15,15,
15,15,15,15,15,15,15,15,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0};


// PWM output (OCR1A)
int LED = 9;

// 16 bit accumulator
uint16_t phaseAccumulator1 = 0;
// 16 bit delta
uint16_t phaseIncrement1 = 0;
uint64_t phaseIncr641;
// DDS resolution
const uint32_t resolution =  /*137439;*/ 68719;
// wavetable lookup index(upper 8 bits of the accumulator)
uint8_t index1 = 0;
// Desired output frequency (let's set it to 420Hz)
uint16_t frequency1 = 90;
unsigned int freqrefresh = 0;
uint8_t volume = 1;

uint16_t frequency2 = 91;
uint16_t phaseAccumulator2 = 0;
uint16_t phaseIncrement2 = 0;
uint64_t phaseIncr642;
uint8_t index2 = 0;

uint16_t frequency3 = 35;
uint16_t phaseAccumulator3 = 0;
uint16_t phaseIncrement3 = 0;
uint64_t phaseIncr643;
uint8_t index3 = 0;

uint8_t center;
uint8_t prop1 = 127;
uint8_t prop2 = 12;
uint8_t prop3 = 7;

const int ButtonPin = 11;
int ButtonState = 0;
bool Lightsaber = false;

// TIMER1 will overflow at a 62.5KHz(Sampling frequency).
// Updates the OCR1A value and the accumulator.
// Computes the next sample to be sent to the PWM.

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int8_t delta0 = 0;
int8_t delta1 = 0;
int8_t delta2 = 0;
int8_t oldeuler0 = 0;
int8_t oldeuler1 = 0;
int8_t oldeuler2 = 0;
uint8_t movement;
uint16_t sample = 0;
uint8_t timestamp = 0;

int movementmax = 0;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
//VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



ISR(TIMER1_OVF_vect)
{
static int16_t osc = 0;

  // Send oscillator output to LED-PWM and DAC
  if(Lightsaber == true)  {
    OCR1A = 255;
    PORTD = osc;
    //PORTB |= _BV(PB1);
    //digitalWrite(LED, HIGH);
    
    /*if(osc > 150) {
      digitalWrite(LED, HIGH);
    }
    else digitalWrite(LED, LOW);*/
  }
  
  else  {
    PORTD = 0;
    OCR1A = 0;
    //PORTB &= ~_BV(PB1);
    //digitalWrite(LED, LOW);
  }

  // Calculate Frequency
  //setFrequency(/*frequency1*/);
  
  // Update accumulator
  phaseAccumulator1 += phaseIncrement1;
  index1 = phaseAccumulator1 >> 8;
  phaseAccumulator2 += phaseIncrement2;
  index2 = phaseAccumulator2 >> 8;
  phaseAccumulator3 += phaseIncrement3;
  index3 = phaseAccumulator3 >> 8;
  
  // Read oscillator value for next interrupt
  osc = 
  volume * (pgm_read_byte(&sineTable100[index1]) + 
  pgm_read_byte(&sineTable010[index2]) + 
  pgm_read_byte(&squareTable006[index3]) - center);
  
  //osc = osc;
  
  if (osc > 127) {osc = 127;}
  else if (osc < -128) {osc = -128;}
  
  osc = osc + 128;
  
  freqrefresh++;
  
  // Tone Demo
  //pgm_read_byte(&sineTable[index2]);

}

// Configures TIMER1 to fast PWM non inverted mode.
// Prescaler set to 1, which means that timer overflows
// every 16MHz/256 = 62.5KHz
void initPWM(void)
{
// Set PORTB1 pin as output
pinMode(LED, OUTPUT);

// 8-bit Fast PWM - non inverted PWM
TCCR1A= _BV(COM1A1) | _BV(WGM10);

// Start timer without prescaler
TCCR1B = _BV(CS10) | _BV(WGM12);

// Enable overflow interrupt for OCR1A
TIMSK1 = _BV(TOIE1);

}

// Translates the desired output frequency to a phase
// increment to be used with the phase accumulator.
// The 16 bit shift is required to remove the  2^16
// scale factor of the resolution.
void setFrequency(/*uint16_t frequencyLOW*/ )
{
phaseIncr641 = resolution * frequency1;
phaseIncrement1 = phaseIncr641 >> 16;

phaseIncr642 = resolution * frequency2;
phaseIncrement2 = phaseIncr642 >> 16;

phaseIncr643 = resolution * frequency3;
phaseIncrement3 = phaseIncr643 >> 16;

}

void LightsaberSwitch() {
    ButtonState = digitalRead(ButtonPin);
  if(ButtonState == HIGH && Lightsaber == false)  {
    Lightsaber = true;
    delay(200);
  }
  else if(ButtonState == HIGH && Lightsaber == true)   {
    Lightsaber = false;
    delay(200);
  }
  
  /*Serial.print(ButtonState);
  Serial.print("   ");
  Serial.print(Lightsaber);
  Serial.print("   \t");*/
}


void setup(void)
{
// Initialise fast PWM
initPWM();

center = 128 * (prop1 + prop2 + prop3)/100;

Serial.begin(115200);

// Set phase increment according to
// desired output frequency
setFrequency(/*requency1*/);

DDRD = 255;

pinMode(ButtonPin, INPUT);

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// set prescale to 16
sbi(ADCSRA,ADPS2) ;
cbi(ADCSRA,ADPS1) ;
cbi(ADCSRA,ADPS0) ;

// Enable global interrupts
sei();

// join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    //Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    /* wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again*/

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    //pinMode(LED_PIN, OUTPUT);

}

void loop()
{
// Go to sleep
//sleep_mode();

if(freqrefresh = 1) LightsaberSwitch();

if(freqrefresh = 20) {

  while (fifoCount < packetSize) {
      //do stuff
      //Serial.println("nix");
      fifoCount = mpu.getFIFOCount();
                       }
    
    if (fifoCount == 1024) mpu.resetFIFO(); 
    
    else {
        fifoCount = mpu.getFIFOCount();
        
        mpu.getFIFOBytes(fifoBuffer, packetSize);
  
        mpu.resetFIFO(); 
        
        fifoCount -= packetSize;
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
  
  }

if(freqrefresh = 50)  delta0 = oldeuler0 - int(euler[0]*100);
//if(freqrefresh = 26)  delta1 = oldeuler1 - int(euler[1]*100);
if(freqrefresh = 80)  delta2 = oldeuler2 - int(euler[2]*100);
if(freqrefresh = 100)  movement =abs(delta0)+abs(delta2);//*abs(delta2);

/*if(freqrefresh = 28) {
  Serial.print(movement);// * 180/M_PI));
  Serial.print("\t");
  }*/



if(freqrefresh = 140)      frequency1 = map(movement, 0, 200, 90, 130);
if(freqrefresh = 200)      frequency2 = map(movement, 0, 200, 91, 130);
if(freqrefresh = 220)      volume = map(movement,0, 200, 1, 10);

if(freqrefresh = 250)  oldeuler0 = int(euler[0]*100);
//if(freqrefresh = 86)  oldeuler1 = int(euler[1]*100);
if(freqrefresh = 300)  oldeuler2 = int(euler[2]*100);

if(freqrefresh = 500) {
   setFrequency();
   freqrefresh = 0;
   /*if(movementmax < movement) movementmax = movement;
   Serial.print(movementmax);
   Serial.print("\t");
   Serial.println(movement);*/
}

      // Frequency Monitor
      //Serial.print(poti);
      //Serial.print("  ");
      //Serial.println(frequency1);
      //Serial.print("  ");
      //Serial.println(phaseIncrement);
}
}
