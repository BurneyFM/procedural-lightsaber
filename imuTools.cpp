#include "Arduino.h"
#include "MPU6050.h"
#include "imuTools.h"

imuTools::imuTools() {
    MPU6050 imu;
}

void imuTools::initialize() {
    imu.initialize();
}

bool imuTools::testConnection() {
    return imu.testConnection();
}

void imuTools::setOffsets(int x, int y, int z) {
    imu.setXGyroOffset(x);
    imu.setYGyroOffset(y);
    imu.setZGyroOffset(z);
}

void imuTools::getSwingIntensity() {
    imu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);

    // TODO get correct values
    // TODO filter irrelevant movements
    // TODO combine into linear motion index
}