/**
 * Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

// #include <iostream>
// #include <cmath>
#include "pid.h"

using namespace std;

class PIDImpl
{
public:
  PIDImpl(float dt, float min, float max, float Kp, float Ki, float Kd);
  ~PIDImpl();
  float calculate(float currentValue, float setPoint);

private:
  float _dt;
  float _min;
  float _max;
  float _Kp;
  float _Ki;
  float _Kd;
  float _pre_error;
  float _integral;
};


PID::PID(float dt, float min, float max, float Kp, float Ki, float Kd) {
  pimpl = new PIDImpl(dt, min, max, Kp, Ki, Kd);
  windowStartTime = millis();
}
float PID::calculate(float currentValue, float setPoint) {
  return pimpl->calculate(currentValue, setPoint);
}
PID::~PID() = default;


/**
 * Implementation
 */
PIDImpl::PIDImpl(float dt, float min, float max, float Kp, float Ki, float Kd) :
  _dt(dt),
  _min(min),
  _max(max),
  _Kp(Kp),
  _Ki(Ki),
  _Kd(Kd),
  _pre_error(0.f),
  _integral(0.f) {
}

float PIDImpl::calculate(float currentValue, float setPoint) {

  // Calculate error
  float error = setPoint - currentValue;

  // Proportional term
  float Pout = _Kp * error;

  // Integral term
  _integral += error * _dt;
  float Iout = _Ki * _integral;

  // Derivative term
  float derivative = 0;
  if(_dt != 0) {
     derivative = (error - _pre_error) / _dt;
  }
  
  float Dout = _Kd * derivative;

  // Calculate total output
  float output = Pout + Iout + Dout;

  // Restrict to max/min
  if (output > _max)
    output = _max;
  else if (output < _min)
    output = _min;

  // Save error to previous error
  _pre_error = error;

  return output;
}

PIDImpl::~PIDImpl() = default;

#endif
