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
    // if brew temp is 92 C, hard limit temp 89 -92.5 C, this is good for pid overshoot, but prone to undershoot
    // after turing PID, add this limit.
    // gaggia boiler system inertia is less than 3C
    // if (currentTemp < brewTempSetPoint - 3.f) {
    //   turnOnBoiler(heatState);
    // }
    // else if (currentTemp > brewTempSetPoint + 0.5f) { 
    //   turnOffBoiler(heatState);
    // }
    // else {
      computeThermoCompensateEnergyByInletWater(INLET_WATER_TEMP, brewTempSetPoint, currentState, heatState, HEAT_BREW_TIME_INTERVAL);
      if (heatState.heatBalancePool > 0.f) {
        computeHeaterConsumedEnergyAndDoHeat(heatState,  currentState.temperature, brewTempSetPoint, HEAT_BREW_TIME_INTERVAL);
      }
      else {
        //reset the heat balance 
        heatState.heatBalancePool = 0.f;
        doPIDAdjust(brewTempSetPoint, onBrewPid, currentState, heatState);
      }
    // }
  }
  else { //if brewState == false
    // if brew temp is 92 C, hard limit temp 82-95 C
    if (currentTemp < brewTempSetPoint - 10.f) {
      turnOnBoiler(heatState);
    }
    else if (currentTemp > brewTempSetPoint + 3.f) {
      turnOffBoiler(heatState);
    }
    else {
      doPIDAdjust(brewTempSetPoint, offBrewPid, currentState, heatState);
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
