/* 21:26 27/12/2023 - change triggering comment */
/* PID control and thermo compensation*/


#ifndef THERMO_CONTROLL_H
#define THERMO_CONTROLL_H


#include "utils.h"
#include "../peripherals/peripherals.h"
#include "../peripherals/pump.h"
#include "../eeprom_data/eeprom_data.h"
#include "sensors_state.h"
#include "heat_state.h"
#include "PID_v1.h"
#include <Arduino.h>


const float HEATER_POWER = 1150.f;            // Gaggia Classic Pro (MODEL: RI9480/SIN035UR) is acually about 1150 Watt
const float WATER_TEMP_RISE_POWER = 4.2f;     // 1ml water rise 1C need 4.2 Joule == 4.2 Watt*Second
const float DEFAULT_GUARANTEE_TEMP = 80.f;   // Default is the lowest guarantee temp
const float DEFAULT_GUARANTEE_TEMP_BREW = 88.f;   // Default is the lowest guarantee temp
const float COLD_WATER_TEMP = 80.f;           // adjust this temp for thermo compensation.
const int HEAT_TIME_INTERVAL = 100;            // ms
const int HEAT_BREW_TIME_INTERVAL = 10;       // ms
const int MAX_BOILER_ON_TIME = 1000;        // ms


void myPIDsInit();

void initOnBrewPID();

void initOffBrewPID();

PID& getOnBrewPID();

PID& getOffBrewPID();

float computeThermoCompensateEnergy(float coldWaterTemp, float targetTemp, const SensorState& currentState, HeatState& heatState, int timeInterval);

float doPIDAdjust(float targetTemp, PID& pidController, const SensorState& currentState, HeatState& heatState, int timeInterval);

void pulseHeaters(HeatState& heatState);

float computeHeaterWastedEnergy(HeatState& heatState, int timeInterval);

void driveHeaterByEnergyBalance(HeatState& heatState, int timeInterval);

void turnOnBoiler(HeatState& heatState);

void turnOffBoiler(HeatState& heatState);

#endif
