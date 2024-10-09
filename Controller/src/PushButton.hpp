#pragma once
#include <Arduino.h>

class PushButton{
  private:
    int _pin;
    String _name;
    bool _buttonState;
    bool _lastButtonState;
    bool _longPressState;
    unsigned long _lastDebounceTime;
    unsigned long _pressedTime;
    unsigned long _debounceWindow = 5; // [ms] ajust depending on the buttons
    unsigned long _longPressTime = 2000; // [ms]
  public:
    void init(int pin, String name);
    int getChangeState();
    bool getState();
    void setDebounceWindow(unsigned long time);
    void setLongPressTime(unsigned long time);
    bool getLongPressState();
};