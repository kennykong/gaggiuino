/* 09:32 15/03/2023 - change triggering comment */
#include "just_do_coffee_better.h"
#include "../lcd/lcd.h"

extern unsigned long steamTime;


void justDoCoffeeBetter(const eepromValues_t &runningCfg, const SensorState &currentState, HeatState &heatState, const bool brewActive) {
  lcdTargetState((int)HEATING::MODE_brew); // setting the target mode to "brew temp"
  float brewTempSetPoint = ACTIVE_PROFILE(runningCfg).setpoint;
  float currentTemp = currentState.temperature;
  PID& offBrewPid = getOffBrewPID();
  PID& onBrewPid = getOnBrewPID();

  if (brewActive) { //if brewState == true
    // hard limit temp 89-95 C
    if (currentTemp < 89.f) {
      turnOnBoiler(heatState);
    }
    else if (currentTemp > 95.f) {
      turnOffBoiler(heatState);
    }
    else {
      computeThermoCompensateEnergy(COLD_WATER_TEMP, brewTempSetPoint, currentState, heatState, HEAT_BREW_TIME_INTERVAL);
      if (heatState.heatBalancePool > 0.f) {
        computeHeaterWastedEnergy(heatState, HEAT_BREW_TIME_INTERVAL);
        driveHeaterByEnergyBalance(heatState, HEAT_BREW_TIME_INTERVAL);
      }
      else {
        doPIDAdjust(brewTempSetPoint, onBrewPid, currentState, heatState, HEAT_BREW_TIME_INTERVAL);
      }
    }
  }
  else { //if brewState == false
    // hard limit temp 80-100 C
    if (currentTemp < 80.f) {
      turnOnBoiler(heatState);
    }
    else if (currentTemp > 100.f) {
      turnOffBoiler(heatState);
    }
    else {
      doPIDAdjust(brewTempSetPoint, offBrewPid, currentState, heatState, HEAT_TIME_INTERVAL);
    }
  }

  if (brewActive || !currentState.brewSwitchState) { // keep steam boiler supply valve open while steaming/descale only
    setSteamValveRelayOff();
  }
  setSteamBoilerRelayOff();
}


//#############################################################################################
//################################____STEAM_POWER_CONTROL____##################################
//#############################################################################################
void steamCtrl(const eepromValues_t &runningCfg, SensorState &currentState) {
  currentState.steamSwitchState ? lcdTargetState((int)HEATING::MODE_steam) : lcdTargetState((int)HEATING::MODE_brew); // setting the steam/hot water target temp
  // steam temp control, needs to be aggressive to keep steam pressure acceptable
  float steamTempSetPoint = runningCfg.steamSetPoint + runningCfg.offsetTemp;
  float sensorTemperature = currentState.temperature + runningCfg.offsetTemp;

  if (currentState.smoothedPressure > steamThreshold_ || sensorTemperature > steamTempSetPoint) {
    setBoilerOff();
    setSteamBoilerRelayOff();
    setSteamValveRelayOff();
    setPumpOff();
  } else {
    if (sensorTemperature < steamTempSetPoint) {
      setBoilerOn();
    } else {
      setBoilerOff();
    }
    setSteamValveRelayOn();
    setSteamBoilerRelayOn();
    #ifndef DREAM_STEAM_DISABLED // disabled for bigger boilers which have no  need of adding water during steaming
      if (currentState.smoothedPressure < activeSteamPressure_) {
        setPumpToRawValue(3);
      } else {
        setPumpOff();
      }
    #endif
  }

  /*In case steam is forgotten ON for more than 15 min*/
  if (currentState.smoothedPressure > passiveSteamPressure_) {
    currentState.isSteamForgottenON = millis() - steamTime >= STEAM_TIMEOUT;
  } else steamTime = millis();
}

/*Water mode and all that*/
void hotWaterMode(const SensorState &currentState) {
  closeValve();
  setPumpToRawValue(80);
  setBoilerOn();
  if (currentState.temperature < MAX_WATER_TEMP) setBoilerOn();
  else setBoilerOff();
}
