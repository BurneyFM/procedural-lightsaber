#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// pins
#define pinLED A3
#define pinSound A0
#define pinButton 13
#define batteryPin A4

//sine LUT
#define waveform 2
#define sineIndex 256

const uint32_t waveformTable[waveform][sineIndex] = {
  //sine
  {
0x80,0x83,0x86,0x89,0x8c,0x8f,0x92,0x95,
0x98,0x9c,0x9f,0xa2,0xa5,0xa8,0xab,0xae,
0xb0,0xb3,0xb6,0xb9,0xbc,0xbf,0xc1,0xc4,
0xc7,0xc9,0xcc,0xce,0xd1,0xd3,0xd5,0xd8,
0xda,0xdc,0xde,0xe0,0xe2,0xe4,0xe6,0xe8,
0xea,0xec,0xed,0xef,0xf0,0xf2,0xf3,0xf5,
0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfc,
0xfd,0xfe,0xfe,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xfe,0xfe,
0xfd,0xfc,0xfc,0xfb,0xfa,0xf9,0xf8,0xf7,
0xf6,0xf5,0xf3,0xf2,0xf0,0xef,0xed,0xec,
0xea,0xe8,0xe6,0xe4,0xe2,0xe0,0xde,0xdc,
0xda,0xd8,0xd5,0xd3,0xd1,0xce,0xcc,0xc9,
0xc7,0xc4,0xc1,0xbf,0xbc,0xb9,0xb6,0xb3,
0xb0,0xae,0xab,0xa8,0xa5,0xa2,0x9f,0x9c,
0x98,0x95,0x92,0x8f,0x8c,0x89,0x86,0x83,
0x80,0x7c,0x79,0x76,0x73,0x70,0x6d,0x6a,
0x67,0x63,0x60,0x5d,0x5a,0x57,0x54,0x51,
0x4f,0x4c,0x49,0x46,0x43,0x40,0x3e,0x3b,
0x38,0x36,0x33,0x31,0x2e,0x2c,0x2a,0x27,
0x25,0x23,0x21,0x1f,0x1d,0x1b,0x19,0x17,
0x15,0x13,0x12,0x10,0x0f,0x0d,0x0c,0x0a,
0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x03,
0x02,0x01,0x01,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,
0x02,0x03,0x03,0x04,0x05,0x06,0x07,0x08,
0x09,0x0a,0x0c,0x0d,0x0f,0x10,0x12,0x13,
0x15,0x17,0x19,0x1b,0x1d,0x1f,0x21,0x23,
0x25,0x27,0x2a,0x2c,0x2e,0x31,0x33,0x36,
0x38,0x3b,0x3e,0x40,0x43,0x46,0x49,0x4c,
0x4f,0x51,0x54,0x57,0x5a,0x5d,0x60,0x63,
0x67,0x6a,0x6d,0x70,0x73,0x76,0x79,0x7c},
{
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,
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
0,0,0,0,0,0,0,0}
};

uint32_t sampleRate = 62500; //sample rate of the sine wave in Hertz, how many times per second the TC5_Handler() function gets called per second basically

//DDS
// 16 bit accumulator
uint16_t phaseAccumulator1 = 0;
uint16_t phaseAccumulator2 = 0;
uint16_t phaseAccumulator3 = 0;
// 16 bit delta
uint16_t phaseIncrement1 = 0;
uint16_t phaseIncrement2 = 0;
uint16_t phaseIncrement3 = 0;
// DDS resolution
const uint32_t resolution =  68719;
// wavetable lookup sample(upper 8 bits of the accumulator)
uint8_t sample1 = 0;
uint8_t sample2 = 0;
uint8_t sample3 = 0;

uint64_t phaseIncr641;
uint64_t phaseIncr642;
uint64_t phaseIncr643;
float frequency1 = 90;
float frequency2 = 91;
int frequency3 = 110;
uint8_t prop1 = 100;
uint8_t prop2 = 10;
uint8_t prop3 = 10;
float volume = 1;
uint8_t center;

int16_t osc;

//MPU
MPU6050 mpu;
uint16_t motion;

// DMP variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// motion variables
Quaternion q;           // [w, x, y, z]         quaternion container
float euler[3];         // [psi, theta, phi]    Euler angle container
float eulerLast[3];     // [theta, phi]    last sample
float eulerDelta[3];    // [theta, phi]    deltas since last sample
float swingAngle;       // angledifference relative to initial position

//timer flags
int calcTimer = 0;
volatile bool freqCalc;
volatile bool getIMU;
volatile bool checkBattery;

//saber led
uint8_t saberState = LOW;
uint16_t saberLED;

//UI
uint8_t buttonState;      // this is the button on the hilt
uint8_t buttonReading;           // signal from button
uint8_t buttonLastState;   // used for debouncing
uint8_t buttonDebounce = 50;    // time in ms for debouncing
uint16_t buttonTimer = 0;           // keep track of button press duration

void setup() {

  //analogWriteResolution(10);

  Wire.begin();
  SERCOM3->I2CM.BAUD.bit.BAUD = SystemCoreClock / ( 2 * 400000) - 1 ; // set DMP frequency
  
  tcConfigure(sampleRate); //configure the timer to run at <sampleRate>Hertz
  tcStartCounter(); //starts the timer

  center = 127 * (prop1 + prop2 + prop3)/100;
  
  Serial.begin(115200);
  //while (!Serial);

  initializeDMP();

  // set inital DDS values
  setFrequency();
}

void loop() {
  //tcDisable(); //This function can be used anywhere if you need to stop/pause the timer
  //tcReset(); //This function should be called everytime you stop the timer

  // check button
  buttonReading = digitalRead(pinButton);

  if (buttonState != buttonLastState) buttonTimer = millis();

  if ((millis() - buttonTimer) > buttonDebounce) {

    // could drop this
    if (buttonState != buttonReading) {
      buttonState = buttonReading;
      
      if (buttonState == HIGH && (millis() - buttonTimer) > 1000) {
        saberState = !saberState;
      }
    }
  }

  buttonLastState = buttonState;
  
  /*if (buttonState == HIGH) {
    Serial.println("Button");
    buttonTimer = millis();
    delay(buttonDebounce);
    
    while (buttonState == HIGH) buttonState = digitalRead(4);
    
    buttonTimer = millis() - buttonTimer;

    // put Button Actions here
    if (buttonTimer > 100) {
      
      buttonState = LOW;
      buttonTimer = 0;
    }
  }*/

  
  if(getIMU == true) {
    if (fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
      
    } else {      
      fetchDMPData();
      calculateMotionData();           
    }
    getIMU = false;
  }

  //Update Frequency
  if(freqCalc == true) {
    setFrequency();
    freqCalc = false;
  }

  if(checkBattery == true) {
    // check for battery voltage < 3.0V
    if((analagRead(batteryPin)*2) < 930) {
      batteryLow();
    }
    checkBattery = false;
  }

  //debug
  Serial.println(saberState);

  /*Serial.print("freq1\t");
  Serial.print(frequency1);Serial.print("\t");
  Serial.print("freq2\t");
  Serial.print(frequency2);Serial.print("\t");
  Serial.print("vol\t");
  Serial.println(volume);Serial.print("\t");

  /*Serial.print("swing\t");
  Serial.println(swingAngle);
  Serial.println("\t");*/
   
}

//this function gets called by the interrupt at <sampleRate>Hertz
void TC5_Handler (void) {

  if (saberState == HIGH) {
    // play sound via DAC
    analogWrite(pinSound, osc);  

    // pulse LED via PWM
    digitalWrite(pinLED, saberLED);
  }
  

  calcTimer++;
  
  //update DDS sample
  phaseAccumulator1 += phaseIncrement1;
  sample1 = phaseAccumulator1 >> 8;
  phaseAccumulator2 += phaseIncrement2;
  sample2 = phaseAccumulator2 >> 8;
  phaseAccumulator3 += phaseIncrement3;
  sample3 = phaseAccumulator3 >> 8;


  // !!!! remove divisions with final proportions and introduce new waveformTable (10us less per loop)
  osc = volume * (
  (waveformTable[0][sample1] * prop1/100) + 
  (waveformTable[0][sample2] * prop2/100) + 
  (waveformTable[0][sample3] * prop3/100) - center);

  if (osc > 127) {osc = 127;}
  else if (osc < -128) {osc = -128;}

  osc += 128;

  saberLED = 128 + osc/2;

  if(calcTimer == 1) {
    getIMU = true;
  }

  if(calcTimer == 80) {
    freqCalc = true;
  }

  if(calcTimer == 160) {
    checkBattery = true;
    calcTimer = 0;
  }
  
  
  // END OF YOUR CODE
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
}

void initializeDMP() {
  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(22);
  mpu.setYGyroOffset(-4);
  mpu.setZGyroOffset(-56);
  mpu.setXAccelOffset(-3);
  mpu.setYAccelOffset(1478);
  mpu.setZAccelOffset(1325);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

   
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

  // set sample rate to 50Hz (1000/(1+19))
  mpu.setRate(19);
  Serial.print("sample rate is ");
  Serial.print(1000/(1+mpu.getRate()));
  
}

void fetchDMPData() {
  // wait for correct available data length, should be a VERY short wait   
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  
  // track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;
  // request DMP calculations
  
  mpu.dmpGetQuaternion(&q, fifoBuffer);

  // DMP get Euler
  mpu.dmpGetEuler(euler, &q);
  
}

void calculateMotionData() {
  // generate Euler deltas for theta and phi, calculate swing angle
  for(int e=1; e<3; e++) {
      euler[e] = abs(euler[e] * 180/M_PI);
      eulerDelta[e] = eulerLast[e] - euler[e];
      eulerLast[e] = euler[e];
  }

  swingAngle = sqrt((eulerDelta[1] * eulerDelta[1]) + (eulerDelta[2] * eulerDelta[2])); 

}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setFrequency(){

  frequency1 = mapFloat(swingAngle, 0, 10, 90, 130);
  frequency2 = mapFloat(swingAngle, 0, 10, 91, 131);
  volume = mapFloat(swingAngle, 0, 10, 1, 10);
  
  phaseIncr641 =  resolution * frequency1;
  phaseIncrement1 = phaseIncr641 >> 16;

  phaseIncr642 = resolution * frequency2;
  phaseIncrement2 = phaseIncr642 >> 16;

  phaseIncr643 = resolution * frequency3;
  phaseIncrement3 = phaseIncr643 >> 16;
}

void batteryLow() {
  
}

/* 
 *  TIMER SPECIFIC FUNCTIONS FOLLOW
 *  you shouldn't change these unless you know what you're doing
 */

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.
 void tcConfigure(int sampleRate)
{
 // Enable GCLK for TCC2 and TC5 (timer counter input clock)
 GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
 while (GCLK->STATUS.bit.SYNCBUSY);

 tcReset(); //reset TC5

 // Set Timer counter Mode to 16 bits
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
 // Set TC5 mode as match frequency
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
 //set prescaler and enable TC5
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
 //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
 TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
 while (tcIsSyncing());
 
 // Configure interrupt request
 NVIC_DisableIRQ(TC5_IRQn);
 NVIC_ClearPendingIRQ(TC5_IRQn);
 NVIC_SetPriority(TC5_IRQn, 0);
 NVIC_EnableIRQ(TC5_IRQn);

 // Enable the TC5 interrupt request
 TC5->COUNT16.INTENSET.bit.MC0 = 1;
 while (tcIsSyncing()); //wait until TC5 is done syncing 
} 

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5 
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}




