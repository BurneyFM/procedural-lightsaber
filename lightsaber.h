#ifndef synthesizer_h
#define synthesizer_h

#include "waveformTables.h"

#ifndef cbi
    #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
    #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


class FrequencySet {
    public:
        FrequencySet(uint16_t frequencies[]) {
            
            int frequencyCount = 0;

            for (uint16_t frequency : frequencies) {
                
                frequencyCount++;
                baseFrequency[frequencyCount] = frequency;

            }

            uint16_t currentFrequency[frequencyCount];
            uint16_t phaseAccumulator[frequencyCount];
            uint16_t phaseIncrement[frequencyCount];
            uint64_t phaseIncr64[frequencyCount];
            uint8_t index[frequencyCount];
            
        };

        uint16_t currentFrequency[];
        uint16_t phaseAccumulator[];
        uint16_t phaseIncrement[];
        uint64_t phaseIncr64[];
        uint8_t index[];
        int frequencyCount;

    private:
        uint16_t baseFrequencies[];

};


class Lightsaber {
    public:
        lightsaber();
        void initialize();
        void setBaseFrequencies();
        void updateLightsaberState();
        int getSampleCount();
        void resetSampleCount();
        bool isEnabled = 0;
    private:
        //int _baseFreqencies[];
        int _sampleCount;
        void _initPWM();
        static int16_t _osc = 0;
        int _center;
        const uint32_t _resolution = 68719;
};


#endif