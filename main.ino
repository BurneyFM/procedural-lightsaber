#include "I2Cdev.h"
#include "MPU6050.h"
#include "imuTools.h"
#include "lightsaber.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define LED A4

const uint16_t baseFrequencies[] = {90, 91, 35}

//MPU6050 accelgyro;
imuTools imu;
Lightsaber lightsaber;

ISR(TIMER1_OVF_vect) {
    lightsaber.updateLightsaberState();
}

void setup() {
    // join I2C bus
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(1000000L);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // set up imu
    imu.initialize();
    imu.setOffsets(220, 76, -85);

    // set up lightsaber
    lightsaber.initialize();
    lightsaber.setBaseFrequencies(baseFrequencies);
}

void loop() {
    imu.getSwingIntensity();

    // finally reset sampleCount
    lightsaber.resetSampleCount();
}
