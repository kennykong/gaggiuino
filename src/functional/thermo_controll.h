/* 21:26 27/12/2023 - change triggering comment */
/* PID control and thermo compensation*/


#ifndef THERMO_CONTROLL_H
#define THERMO_CONTROLL_H

#include "../peripherals/peripherals.h"
#include "sensors_state.h"
#include "heat_state.h"
#include "PID_v2.h"
// #include "log.h"
#include <Arduino.h>


const float HEATER_POWER = 1150.f;            // Gaggia Classic Pro (MODEL: RI9480/SIN035UR) is acually about 1150 Watt
const float WATER_TEMP_RISE_POWER = 4.2f;     // 1ml water rise 1C need 4.2 Joule == 4.2 Watt*Second
// const float INLET_WATER_TEMP = 69.f;          // adjust this temp for thermo compensation. //deprecated
const int HEAT_TIME_INTERVAL = 100;           // ms, pid interval off brew
const int HEAT_BREW_TIME_INTERVAL = 10;       // ms, pid interval on brew
const float MAX_BOILER_ON_PERCENTAGE = 100.f;     // %, pid output max, 100% heater power
const int TEMP_CHECK_INTERVAL = 400;          // ms, when temp drop to 1C/S. //deprecated


void myPIDsInit();

void initOnBrewPID();

void initOffBrewPID();

void resetThemoCompState(HeatState& heatState, const SensorState& currentState);

PID& getOnBrewPID();

PID& getOffBrewPID();

float computeThermoCompensateEnergyByInletWater(float compensateTemp, float targetTemp, const SensorState& currentState, HeatState& heatState, int timeInterval);

float computeHeaterConsumedEnergyAndDoHeat(HeatState& heatState, float currentTemp, float setPoint, float upperLimit, float downLimit, int timeInterval);

void driveHeaterByEnergyBalance(HeatState& heatState, float currentTemp, float setPoint, float upperLimit, float downLimit);

float caculateCorrection(HeatState& heatState, float currentTemp, float setPoint);

double doPIDAdjust(float targetTemp, PID& pidController, const SensorState& currentState, HeatState& heatState);

double doPIDAdjustWithLimit(float targetTemp, float downLimit, float upperLimit, PID& pidController, const SensorState& currentState, HeatState& heatState);

void pulseHeaters(HeatState& heatState, unsigned long powerAdjustCycle);

void turnOnBoiler(HeatState& heatState);

void turnOffBoiler(HeatState& heatState);

void turnOnBoiler(HeatState& heatState, bool isBoilerOperatorTC);

void turnOffBoiler(HeatState& heatState, bool isBoilerOperatorTC);

#endif
