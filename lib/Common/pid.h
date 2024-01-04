// /**
//  * Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>
//  *
//  * Permission is hereby granted, free of charge, to any person obtaining a copy
//  * of this software and associated documentation files (the "Software"), to deal
//  * in the Software without restriction, including without limitation the rights
//  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  * copies of the Software, and to permit persons to whom the Software is
//  * furnished to do so, subject to the following conditions:
//  *
//  * The above copyright notice and this permission notice shall be included in
//  * all copies or substantial portions of the Software.
//  *
//  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  * THE SOFTWARE.
//  */

// #ifndef _PID_H_
// #define _PID_H_

// #include <Arduino.h>

// class PIDImpl;
// class PID
// {
// public:
//   // dt -  loop interval time (second)
//   // min - minimum value of manipulated variable
//   // max - maximum value of manipulated variable
//   // Kp -  proportional gain
//   // Ki -  Integral gain
//   // Kd -  derivative gain
//   PID(double dt, double min, double max, double Kp, double Ki, double Kd);

//   // Returns the manipulated variable given a setpoint and current process value
//   double calculate(double currentValue, double setPoint);

//   //for relay on-off use
//   uint32_t windowStartTime;

//   ~PID();

// private:
//   PIDImpl* pimpl;
// };

// #endif
