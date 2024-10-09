#pragma once
#include <Arduino.h>

class Joystick{
  private:
    int _pinX;
    int _pinY;
    int _xAxisMax = 10;
    int _yAxisMax = 10;
    int _xAxisMin = -10;
    int _yAxisMin = -10;
    int _xValue = 0;
    int _yValue = 0;
    int _xPreviousValue = 0;
    int _yPreviousValue = 0;
    bool _valueChangeFlag = false;
    int _changeFlagThreshold = 1; // judghe the flag when it is over the threshold
    int _xOffset = 0;
    int _yOffset = 0;
    int _meanSize = 100; // for measuring an offset value
  public:
    void setPins(int pinX, int pinY);
    void setRange(int xAxisMin, int xAxisMax, int yAxisMin, int yAxisMax);
    void setMeanSize(int size);
    void setThreshold(int threshold);
    void updateOffset();
    void updateValue();
    int getX();
    int getY();
    bool getValueChangeFlag();
};

