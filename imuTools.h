#ifndef imuTools_h
#define imuTools_h

class imuTools {
    public:
        imuTools();
        void initialize();
        bool testConnection();
        void setOffsets(int x, int y, int z);
        void getSwingIntensity();
    private:
        MPU6050 imu;
        int16_t _ax;
        int16_t _ay;
        int16_t _az;
        int16_t _gx;
        int16_t _gy;
        int16_t _gz;
};

#endif