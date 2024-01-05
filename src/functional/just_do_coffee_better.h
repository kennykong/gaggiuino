/* 09:32 15/03/2023 - change triggering comment */
#ifndef JUST_DO_COFFEE_BETTER_H
#define JUST_DO_COFFEE_BETTER_H

#include "utils.h"
#include "../peripherals/peripherals.h"
#include "../peripherals/pump.h"
#include "../eeprom_data/eeprom_data.h"
#include "sensors_state.h"
#include "thermo_controll.h"
#include <Arduino.h>


const uint32_t STEAM_TIMEOUT = 900000UL; // steam mode timeout
const float MAX_WATER_TEMP = 105.f; // max temp allowed in hot water mode
const float steamThreshold_ = 11.f; // max allowed pressure before heaters are disabled
const float activeSteamPressure_ = 2.f; // DreamSteam enabling threshold.
const float passiveSteamPressure_ = 3.f; // DreamSteam disabling threshold.

enum class HEATING {
  MODE_brew,
  MODE_steam,
  MODE_hotWater
};

void justDoCoffeeBetter(const eepromValues_t &runningCfg, const SensorState &currentState, HeatState &heatState, const bool brewActive);
void steamCtrl(const eepromValues_t &runningCfg, SensorState &currentState);
void hotWaterMode(const SensorState &currentState);

#endif

