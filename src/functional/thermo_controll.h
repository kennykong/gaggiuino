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
#include "MiniPID.h"
#include <Arduino.h>


const float HEATER_POWER = 1200.f;            // Gaggia Classic Pro (MODEL: RI9480/SIN035UR) is 1200 Watt
const float WATER_TEMP_RISE_POWER = 4.2f;     // 1ml water rise 1C need 4.2 Joule == 4.2 Watt*Second
const float DEFAULT_COLD_WATER_TEMP = 20.f;   // Default is average air temperature of the year. Maybe set from UI or given by sensor later.
const int HEAT_TIME_INTERVAL = 20;            // ms
const int HEAT_BREW_TIME_INTERVAL = 5;        // ms

void allPIDsInit();

void allPIDsReset();

void offBrewPIDInit();

void onBrewPIDInit();

MiniPID& getOnBrewPID();

MiniPID& getOffBrewPID();

float computeThermoCompensateEnergy(float coldWaterTemp, float targetTemp, const SensorState& currentState, HeatState& heatState, int timeInterval);

float computePIDAdjustEnergy(float targetTemp, MiniPID& pidController, const SensorState& currentState, HeatState& heatState, int timeInterval);

float computeHeaterWastedEnergy(HeatState& heatState, int timeInterval);

void driveHeaterByEnergyBalance(HeatState& heatState, int timeInterval);

#endif
