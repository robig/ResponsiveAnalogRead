/*
 * ResponsiveAnalogRead.cpp
 * Arduino library for eliminating noise in analogRead inputs without decreasing responsiveness
 *
 * Copyright (c) 2016 Damien Clarke
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include "ResponsiveAnalogRead.h"

void ResponsiveAnalogRead::begin(int pin, bool sleepEnable, float snapMultiplier){
    pinMode(pin, INPUT ); // ensure button pin is an input
    digitalWrite(pin, LOW ); // ensure pullup is off on button pin
    
    this->pin = pin;
    this->sleepEnable = sleepEnable;
    setSnapMultiplier(snapMultiplier);

    analogReadResolution(10);
    _max=1023; //10 bits
    //_max = 4095; //ESP has 12bits on ADC pins (see https://www.arduino.cc/reference/en/language/functions/zero-due-mkr-family/analogreadresolution/)

    _useByte=true; // already map analogRead value to byte
}


void ResponsiveAnalogRead::update()
{
  rawValue = _useByte? doMapping(analogRead(pin)) : analogRead(pin);
  this->update(rawValue);
}

void ResponsiveAnalogRead::update(int rawValueRead)
{
  rawValue = rawValueRead;
  prevResponsiveValue = responsiveValue;
  responsiveValue = getResponsiveValue(rawValue);
  responsiveValueHasChanged = responsiveValue != prevResponsiveValue;
  if(_debug && responsiveValueHasChanged) {
    Serial.print(F("Change: raw=")); Serial.print(rawValue); Serial.print(F(" responsiveValue=")); Serial.println(responsiveValue);
  }
}

int ResponsiveAnalogRead::getResponsiveValue(int newValue)
{
  // if sleep and edge snap are enabled and the new value is very close to an edge, drag it a little closer to the edges
  // This'll make it easier to pull the output values right to the extremes without sleeping,
  // and it'll make movements right near the edge appear larger, making it easier to wake up
  if(sleepEnable && edgeSnapEnable) {
    if(newValue < activityThreshold) {
      newValue = (newValue * 2) - activityThreshold;
    } else if(newValue > analogResolution - activityThreshold) {
      newValue = (newValue * 2) - analogResolution + activityThreshold;
    }
  }

  // get difference between new input value and current smooth value
  unsigned int diff = abs(newValue - smoothValue);

  // measure the difference between the new value and current value
  // and use another exponential moving average to work out what
  // the current margin of error is
  errorEMA += ((newValue - smoothValue) - errorEMA) * 0.4;

  // if sleep has been enabled, sleep when the amount of error is below the activity threshold
  if(sleepEnable) {
    // recalculate sleeping status
    sleeping = abs(errorEMA) < activityThreshold;
  }

  // if we're allowed to sleep, and we're sleeping
  // then don't update responsiveValue this loop
  // just output the existing responsiveValue
  if(sleepEnable && sleeping) {
    return (int)smoothValue;
  }

  // use a 'snap curve' function, where we pass in the diff (x) and get back a number from 0-1.
  // We want small values of x to result in an output close to zero, so when the smooth value is close to the input value
  // it'll smooth out noise aggressively by responding slowly to sudden changes.
  // We want a small increase in x to result in a much higher output value, so medium and large movements are snappy and responsive,
  // and aren't made sluggish by unnecessarily filtering out noise. A hyperbola (f(x) = 1/x) curve is used.
  // First x has an offset of 1 applied, so x = 0 now results in a value of 1 from the hyperbola function.
  // High values of x tend toward 0, but we want an output that begins at 0 and tends toward 1, so 1-y flips this up the right way.
  // Finally the result is multiplied by 2 and capped at a maximum of one, which means that at a certain point all larger movements are maximally snappy

  // then multiply the input by SNAP_MULTIPLER so input values fit the snap curve better.
  float snap = snapCurve(diff * snapMultiplier);

  // when sleep is enabled, the emphasis is stopping on a responsiveValue quickly, and it's less about easing into position.
  // If sleep is enabled, add a small amount to snap so it'll tend to snap into a more accurate position before sleeping starts.
  if(sleepEnable) {
    snap *= 0.5 + 0.5;
  }

  // calculate the exponential moving average based on the snap
  smoothValue += (newValue - smoothValue) * snap;

  // ensure output is in bounds
  if(smoothValue < 0.0) {
    smoothValue = 0.0;
  } else if(smoothValue > analogResolution - 1) {
    smoothValue = analogResolution - 1;
  }

  // expected output is an integer
  return (int)smoothValue;
}

float ResponsiveAnalogRead::snapCurve(float x)
{
  float y = 1.0 / (x + 1.0);
  y = (1.0 - y) * 2.0;
  if(y > 1.0) {
    return 1.0;
  }
  return y;
}

void ResponsiveAnalogRead::setSnapMultiplier(float newMultiplier)
{
  if(newMultiplier > 1.0) {
    newMultiplier = 1.0;
  }
  if(newMultiplier < 0.0) {
    newMultiplier = 0.0;
  }
  snapMultiplier = newMultiplier;
}

int ResponsiveAnalogRead::multiMap(int val)
{
  if(_debug) { Serial.printf(" val=%i",val); };
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  if (val >= _in[_mapSize-1]) {
    /*if(_debug) {
      Serial.printf(" for %i exit in=%i #%i to out=%i\n",val, _in[_mapSize-1], _mapSize-1, _out[_mapSize-1]);
      Serial.print("in=");
      for(int i=0;i<_mapSize;i++)
        Serial.printf("%i, ",_in[i]);
      Serial.println();
        Serial.print("out=");
      for(int i=0;i<_mapSize;i++)
        Serial.printf("%i, ",_out[i]);
      Serial.println();
    }*/
    return _out[_mapSize-1];
  }
  if(_debug) { Serial.print(" step2"); }

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;
  if(_debug) { Serial.printf(" for %i found in=%i #%i to out=%i\n",val, _in[pos], pos, _out[pos]); }

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  if(_debug) { Serial.print(" step"); }
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}


void ResponsiveAnalogRead::setMap(int* in, int* out, uint8_t size){
  _in=in; _out=out; _mapSize=size;
  _map=true;
  if(_debug) {
    Serial.print("in=");
    for(int i=0;i<size;i++)
      Serial.printf("%i, ",_in[i]);
    Serial.println();
    Serial.print("out=");
    for(int i=0;i<size;i++)
      Serial.printf("%i, ",_out[i]);
    Serial.println();
    Serial.printf("mapSize=%i\n",_mapSize);
  }
}


int ResponsiveAnalogRead::doMapping(int val) {
  if(_map) 
    return multiMap(val);
  else
    return map(val, _min, _max, _toMin, _toMax);
}

byte ResponsiveAnalogRead::getByteValue() {
  if(_useByte) return getValue(); // is alrady Byte
  return doMapping(getValue());
}

void ResponsiveAnalogRead::calibrate() {

  Serial.printf ("Starting potentiometer calibration on PIN %i:\n", pin);
  Serial.println(F("=============================================="));
  Serial.println();
  Serial.print(F("Turn all the way to the minimum."));
  delay(5000);
  int min = analogRead(pin);
  Serial.printf(" %i\n", min);
  Serial.print(F("Turn all the way to the maximum."));
  delay(5000);
  int max = analogRead(pin);
  Serial.printf(" %i\n", max);
  Serial.println(F("Now turn slowly from minimum to maximum"));
  delay(4000);

  while(analogRead(pin)==0) delay(100);
  
  const int MAX=255;
  int buf[255];
  int pos=1;
  buf[0]=min;
  int _sensorRaw=0;
  while(_sensorRaw<max) {
    _sensorRaw = analogRead(pin);
    Serial.printf(" %i", _sensorRaw);
    buf[pos]=_sensorRaw;
    if(pos>MAX) {
      Serial.println(F("Movement too slow! Please start again."));
      delay(2000);
      return;
    }
    delay(500);
    Serial.print(".");
    pos++;
  }
  Serial.println();
  Serial.printf("Good. Got %i data points.\n", pos+1);

  Serial.print("int in[]={");
  for(int i=0; i<pos; i++){
    Serial.printf("%i,", buf[i]);
  }
  Serial.printf("%i};\n", max);

  Serial.printf("int out[]={");
  int add=255/(pos+1);
  for(int i=0; i<pos; i++){
    Serial.printf("%i,", i*add);
  }
  Serial.printf("%i};\n", 255);
  Serial.printf("int size=%i;\n", pos+1);


  delay(3000);
}