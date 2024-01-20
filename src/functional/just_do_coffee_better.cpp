/* 09:32 15/03/2023 - change triggering comment */
#include "just_do_coffee_better.h"
#include "../lcd/lcd.h"

extern unsigned long steamTime;


void justDoCoffeeBetter(const eepromValues_t &runningCfg, const SensorState &currentState, HeatState &heatState, const bool brewActive) {
  lcdTargetState((int)HEATING::MODE_brew); // setting the target mode to "brew temp"
  float brewTempSetPoint = ACTIVE_PROFILE(runningCfg).setpoint;
  float compensateTemp = runningCfg.compensateTemp;
  float upperLimit = runningCfg.brewUpperLimitTemp; 
  float downLimit = runningCfg.brewDownLimitTemp;
  PID& offBrewPid = getOffBrewPID();
  PID& onBrewPid = getOnBrewPID();

  if (brewActive) { //if brewState == true

    computeThermoCompensateEnergyByInletWater(compensateTemp, brewTempSetPoint, currentState, heatState, HEAT_BREW_TIME_INTERVAL);
    if (heatState.heatBalancePool > 0.f) {
      computeHeaterConsumedEnergyAndDoHeat(heatState, currentState.temperature, brewTempSetPoint, upperLimit, downLimit, HEAT_BREW_TIME_INTERVAL);
    }
    else {
      //reset the heat balance 
      heatState.heatBalancePool = 0.f;
      // doPIDAdjust(brewTempSetPoint, onBrewPid, currentState, heatState);
      doPIDAdjustWithLimit(brewTempSetPoint, downLimit, upperLimit, onBrewPid, currentState, heatState);
    }
  }
  else { //if brewState == false
    // if brew temp is 92 C, hard limit temp 82-95 C
    doPIDAdjustWithLimit(brewTempSetPoint, 10.f, 3.f, offBrewPid, currentState, heatState);
  }

  if (brewActive || !currentState.brewSwitchState) { // keep steam boiler supply valve open while steaming/descale only
    setSteamValveRelayOff();
  }
  setSteamBoilerRelayOff();
}


//#############################################################################################
//################################____STEAM_POWER_CONTROL____##################################
//#############################################################################################
void steamCtrl(const eepromValues_t &runningCfg, SensorState &currentState, HeatState &heatState) {
  currentState.steamSwitchState ? lcdTargetState((int)HEATING::MODE_steam) : lcdTargetState((int)HEATING::MODE_brew); // setting the steam/hot water target temp
  // steam temp control, needs to be aggressive to keep steam pressure acceptable
  float steamTempSetPoint = runningCfg.steamSetPoint;
  float sensorTemperature = currentState.sensorTemperature;

  if (currentState.smoothedPressure > steamThreshold_ || sensorTemperature > steamTempSetPoint + 5.f) {
    setBoilerOff();
    setSteamBoilerRelayOff();
    setSteamValveRelayOff();
    setPumpOff();
  } else {
    
    doPIDAdjustWithLimit(steamTempSetPoint, 5.f, 3.f, getOnBrewPID(), currentState, heatState);
    // if (sensorTemperature < steamTempSetPoint) {
    //   setBoilerOn();
    // } else {
    //   setBoilerOff();
    // }
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
void hotWaterMode(const SensorState &currentState, HeatState &heatState) {
  closeValve();
  setPumpToRawValue(80);
  // setBoilerOn();
  doPIDAdjustWithLimit(MAX_WATER_TEMP, 5.f, 3.f, getOnBrewPID(), currentState, heatState);
  // if (currentState.temperature < MAX_WATER_TEMP) setBoilerOn();
  // else setBoilerOff();
}
