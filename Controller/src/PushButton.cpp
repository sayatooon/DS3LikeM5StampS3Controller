#include "PushButton.hpp"

void PushButton::init(int pin, String name){
  _pin = pin;
  _name = name;
  _buttonState = HIGH;
  _lastButtonState = HIGH;
  _longPressState = false;
  _lastDebounceTime = millis();
}

/* check the push button change state */
// 0: no changed
// 1: pushed
// 2: released
int PushButton::getChangeState(){
  int state = digitalRead(_pin);
    
    if (state != _lastButtonState) { // reset timer
        _lastDebounceTime = millis();
    }
  
    if ((millis() - _lastDebounceTime) > _debounceWindow) {
        if (state != _buttonState) {
            _buttonState = state;          
            if (_buttonState == LOW) { // pushed
                _lastButtonState = state;
                _longPressState = false;
                _pressedTime = millis();
                return 1;
            }
            if (_buttonState == HIGH) { // released
                _lastButtonState = state;
                if ((millis() - _pressedTime) > _longPressTime){
                  _longPressState = true;
                }
                return 2;
            }
        }
    }
    _lastButtonState = state;
    return 0;
}

/* get the push button state */
// HIGH: off
// LOW: on 
bool PushButton::getState(){
  return _buttonState;
}

void PushButton::setDebounceWindow(unsigned long time){
  _debounceWindow = time;
}

void PushButton::setLongPressTime(unsigned long time){
  _longPressTime = time;
}

bool PushButton::getLongPressState(){
  return _longPressState;
}

