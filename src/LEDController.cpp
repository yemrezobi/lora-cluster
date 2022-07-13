#include <Arduino.h>
#include "LEDController.h"

LEDController::LEDController(int pin) : _pin{pin}, _pinState{false}, _arrayLen{0}, _counter{0} {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
}

LEDController::LEDController(int pin, unsigned long periods[], int length, bool repeat) : _pin{pin}, _periods{new unsigned long[length]}, _arrayLen{length}, _counter{0}, _startTime{millis()}, _pinState{true}, _repeat{repeat} {
    memcpy(_periods, periods, length * sizeof(unsigned long));
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, HIGH);
}

LEDController::~LEDController() {
    delete _periods;
}

void LEDController::setPeriods(unsigned long periods[], int length, bool repeat) {
    _periods = new unsigned long[length];
    _arrayLen = length;
    _counter = 0;
    _startTime = millis();
    _pinState = true;
    _repeat = repeat;
    memcpy(_periods, periods, length * sizeof(unsigned long));
    
    digitalWrite(_pin, HIGH);
}

bool LEDController::poll() {
    if(_counter >= _arrayLen){ // end of array
        _pinState = false;
        digitalWrite(_pin, LOW);
        return false;
    }
    if((millis() - _startTime) > _periods[_counter]){
        digitalWrite(_pin, _pinState ? LOW : HIGH); // toggle LED state
        _pinState = !_pinState;
        _startTime = millis();
        if(++_counter >= _arrayLen && _repeat){
            _counter = 0;
        }
    }
    if(_periods[_counter] > 5000ul){ //longer than 5s means something probably went wrong
        _pinState = false;
        digitalWrite(_pin, LOW);
    }
    return true;
}
