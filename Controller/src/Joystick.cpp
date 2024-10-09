#include "Joystick.hpp"

void Joystick::setPins(int pinX, int pinY){
  _pinX = pinX;
  _pinY = pinY;
}

void Joystick::setRange(int xAxisMin, int xAxisMax, int yAxisMin, int yAxisMax){
  _xAxisMin = xAxisMin;
  _xAxisMax = xAxisMax;
  _yAxisMin = yAxisMin;
  _yAxisMax = yAxisMax;
}

void Joystick::setMeanSize(int size){
  _meanSize = size;
}

void Joystick::setThreshold(int threshold){
  _changeFlagThreshold = threshold;
}

void Joystick::updateOffset(){
  _xOffset = 0;
  _yOffset = 0;

  for(int i = 0; i < _meanSize; i++){
    _xOffset += analogRead(_pinX);
    _yOffset += analogRead(_pinY);
    delay(10);
  }

  _xOffset /= _meanSize;
  _yOffset /= _meanSize;
  //_xOffset = map(_xOffset, 0, 4095, _xAxisMin, _xAxisMax);
  //_yOffset = map(_yOffset, 0, 4095, _yAxisMin, _yAxisMax);
}

void Joystick::updateValue(){
  _xPreviousValue = _xValue;
  _yPreviousValue = _yValue;
  _xValue = map(analogRead(_pinX) - _xOffset, 0 - _xOffset, 4095 - _xOffset, _xAxisMin, _xAxisMax);
  _yValue = map(analogRead(_pinY) - _yOffset, 0 - _yOffset, 4095 - _yOffset, _yAxisMin, _yAxisMax);
  if((abs(_xValue) > _changeFlagThreshold || abs(_yValue) > _changeFlagThreshold) && ( _xValue != _xPreviousValue || _yValue != _yPreviousValue)){
    _valueChangeFlag = true;
  }else{
    _valueChangeFlag = false;
  }
}

int Joystick::getX(){
  return _xValue;  
}

int Joystick::getY(){
  return _yValue;
}

bool Joystick::getValueChangeFlag(){
  return _valueChangeFlag;
}