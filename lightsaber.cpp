#include "Arduino.h"
#include "lightsaber.h"

Lightsaber::Lightsaber() {}

void Lightsaber::initialize() {
    
    _initPWM();

    // TODO
    center = ...

    // set prescale to 16
    sbi(ADCSRA,ADPS2);
    cbi(ADCSRA,ADPS1);
    cbi(ADCSRA,ADPS0);

    sei();
}

void Lightsaber::setFrequencies(uint16_t frequencies[]) {

    FrequencySet frequencySet(frequencies);

}

void Lightsaber::prepareSample() {
    /* phaseIncr641 = resolution * frequency1;
    phaseIncrement1 = phaseIncr641 >> 16;

    phaseIncr642 = resolution * frequency2;
    phaseIncrement2 = phaseIncr642 >> 16;

    phaseIncr643 = resolution * frequency3;
    phaseIncrement3 = phaseIncr643 >> 16; */

    for (int i : frequencySet.frequencyCount-1) {
        frequencySet.phaseIncr64[i] = _resolution * frequencySet.currentFrequency[i];
        frequencySet.phaseIncrement[i] = frequencySet.phaseIncr64[i] >> 16;
    }

}

void Lightsaber::updateLightsaberState() {

    if(isEnabled)  {
        OCR1A = 255;
        PORTD = _osc;
    }
    else  {
        PORTD = 0;
        OCR1A = 0;
    }

    // determine sample position in waveform table
    /* phaseAccumulator1 += phaseIncrement1;
    index1 = phaseAccumulator1 >> 8;
    phaseAccumulator2 += phaseIncrement2;
    index2 = phaseAccumulator2 >> 8;
    phaseAccumulator3 += phaseIncrement3;
    index3 = phaseAccumulator3 >> 8; */
    for (int i : frequencySet.frequencyCount-1) {
        frequencySet.index[i] = frequencySet.phaseAccumulator[i] >> 8;
        frequencySet.phaseAccumulator[i] += phaseIncrement[i];
    }


    // Read oscillator value for next interrupt
    osc = volume * (
        pgm_read_byte(&sineTable100[frequencySet.index[1]]) + 
        pgm_read_byte(&sineTable010[frequencySet.index[2]]) + 
        pgm_read_byte(&squareTable006[frequencySet.index[3]]
        ) - _center);
    
    // clamp _osc
    if (osc > 127) osc = 127;
    else if (osc < -128) osc = -128;

    osc = osc + 128;

    _sampleCount++;
}

int Lightsaber::getSampleCount() {
    return _sampleCount;
}

void Lightsaber::resetSampleCount() {
    _sampleCount = 0;
}

void Lightsaber::_initPWM() {
    
    // Set PORTB1 pin as output
    pinMode(LED, OUTPUT);

    // 8-bit Fast PWM - non inverted PWM
    TCCR1A= _BV(COM1A1) | _BV(WGM10);

    // Start timer without prescaler
    TCCR1B = _BV(CS10) | _BV(WGM12);

    // Enable overflow interrupt for OCR1A
    TIMSK1 = _BV(TOIE1);
}