#include "Arduino.h"
#include "lightsaber.h"

Lightsaber::Lightsaber() {}

void Lightsaber::initialize() {
    
    _initPWM();

    // set prescale to 16
    sbi(ADCSRA,ADPS2);
    cbi(ADCSRA,ADPS1);
    cbi(ADCSRA,ADPS0);

    sei();

    // mean value of osc - fixed for now; dynamic center would require dynamic progmem in waveformTables.h
    _center = 128 * (127 + 12 + 7)/100;
}

void Lightsaber::setBaseFrequencies(int frequencies[]) {

    FrequencySet frequencySet(frequencies);

}

void Lightsaber::prepareSample() {
    for (int i = 0; i < frequencySet._frequencyCount; i++) {
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
        OCR1A = 0;
        PORTD = 0;
    }

    // determine sample position in waveform table
    for (int i = 0; i < frequencySet._frequencyCount; i++) {
        frequencySet.phaseAccumulator[i] += phaseIncrement[i];
        frequencySet.index[i] = frequencySet.phaseAccumulator[i] >> 8;
    }


    // Read oscillator value for next interrupt
    _osc = volume * (
        pgm_read_byte(&sineTable100[frequencySet.index[0]]) + 
        pgm_read_byte(&sineTable010[frequencySet.index[1]]) + 
        pgm_read_byte(&squareTable006[frequencySet.index[2]]
        ) - _center);
    
    // clamp _osc
    if (_osc > 127) osc = 127;
    else if (_osc < -128) osc = -128;

    _osc = _osc + 128;

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