#ifndef lightsaber_h
#define lightsaber_h

#include "waveformTables.h"

#ifndef cbi
    #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
    #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define maxNumberFrequencies 3


class FrequencySet {
    public:
        FrequencySet(int frequencies[]) {

            int _frequencyCount = 0;
            
            for (int frequency : frequencies) {
                _frequencyCount++;
            }

            uint16_t currentFrequency[_frequencyCount] = {0};
            uint16_t phaseAccumulator[_frequencyCount] = {0};
            uint16_t phaseIncrement[_frequencyCount] = {0};
            uint64_t phaseIncr64[_frequencyCount] = {0};
            uint8_t index[_frequencyCount] = {0};

            for (int i = 0; i < _frequencyCount; i++) {
                _baseFrequencies[i] = frequencies[i];
                currentFrequency[i] = frequencies[i];
            }

            Serial.print("Lightsaber sound frequencies: ");
            Serial.println(_baseFrequencies);
            
        };

        uint16_t currentFrequency[_frequencyCount];
        uint16_t phaseAccumulator[_frequencyCount];
        uint16_t phaseIncrement[_frequencyCount];
        uint64_t phaseIncr64[_frequencyCount];
        int index[_frequencyCount];

    private:
        int _frequencyCount;
        int _baseFrequencies[];

};


class Lightsaber {
    public:
        lightsaber();
        void initialize();
        void setBaseFrequencies(int frequencies[]);
        void prepareSample();
        void updateLightsaberState();
        int getSampleCount();
        void resetSampleCount();
        bool isEnabled = false;
    private:
        int _sampleCount = 0;
        void _initPWM();
        int16_t _osc = 0;
        int _center;
        const uint32_t _resolution = 68719;
};


#endif