/* 21:26 27/12/2023 - change triggering comment */
/* PID control and thermo compensation*/

#include "thermo_controll.h"
#include "math.h"

namespace {
  class PIDGroup {
  public:
    static MiniPID& getOnBrewPID() {
      static MiniPID onBrewPID(0, 0, 0);
      return onBrewPID;
    };
    static MiniPID& getOffBrewPID() {
      static MiniPID offBrewPID(0, 0, 0);
      return offBrewPID;
    };
  private:
    PIDGroup() = default;
    ~PIDGroup() = default;
  };
}

void allPIDsInit() {
  offBrewPIDInit();
  onBrewPIDInit();
}

void allPIDsReset() {
  PIDGroup::getOffBrewPID().reset();
  PIDGroup::getOnBrewPID().reset();
}

void offBrewPIDInit() {
  MiniPID pid = PIDGroup::getOffBrewPID();

  float offBrewP = 0.5;
  float offBrewI = 0;
  float offBrewD = 0;

  pid.setPID(offBrewP, offBrewI, offBrewD);
  pid.setOutputLimits(0, HEATER_POWER); // max 1s heat
}

void onBrewPIDInit() {
  MiniPID pid = PIDGroup::getOnBrewPID();
  
  float onBrewP = 0.5;
  float onBrewI = 0;
  float onBrewD = 0;

  pid.setPID(onBrewP, onBrewI, onBrewD);
  pid.setOutputLimits(0, HEATER_POWER); // max 1s heat
}

MiniPID& getOnBrewPID() {
  return PIDGroup::getOnBrewPID();
}

MiniPID& getOffBrewPID() {
  return PIDGroup::getOffBrewPID();
}


float computeThermoCompensateEnergy(float coldWaterTemp, float targetTemp, const SensorState& currentState, HeatState& heatState, int timeInterval) {
  
  if (coldWaterTemp == 0.f) {
    coldWaterTemp = DEFAULT_COLD_WATER_TEMP;
  }
  float deltaHeat = 0.f;
  float deltaTemp = targetTemp - coldWaterTemp;
  float currentWaterPumped = currentState.waterPumped;
  float deltaWater = currentWaterPumped - heatState.lastWaterPumped;
  uint32_t currentWaterPumpedTimestamp = millis();
  int deltaTime = currentWaterPumpedTimestamp - heatState.lastWaterPumpedTimestamp;
  if (deltaTime > timeInterval && deltaWater > 0 ) {
    heatState.lastWaterPumped = currentWaterPumped;
    heatState.lastWaterPumpedTimestamp = currentWaterPumpedTimestamp;
    float deltaHeat = deltaWater * deltaTemp * WATER_TEMP_RISE_POWER;
    heatState.thermoCompensateHeat = deltaHeat;

    // add balance
    heatState.heatBalancePool += deltaHeat;
  }

  return deltaHeat;
}


float computePIDAdjustEnergy(float targetTemp, MiniPID& pidController, const SensorState& currentState, HeatState& heatState, int timeInterval) {

  float adjustEnergy = 0.f;
  float currentTemp = currentState.temperature;

  uint32_t currentTimestamp = millis();
  int deltaTime = currentTimestamp - heatState.lastPidAdjustTimestamp;
  if (deltaTime > timeInterval) {
    adjustEnergy = pidController.getOutput(currentTemp, targetTemp);
    heatState.pidAdjustHeat = adjustEnergy;
    heatState.lastPidAdjustTimestamp = currentTimestamp;
    // add balance
    heatState.heatBalancePool += adjustEnergy;
  }

  return adjustEnergy;
}


float computeHeaterWastedEnergy(HeatState& heatState, int timeInterval) {
  float energyWasted = 0.f;
  uint32_t currentTimestamp = millis();
  
  int deltaTime = currentTimestamp - heatState.lastBoilerStateTimestamp;
  if (deltaTime > timeInterval && heatState.lastBoilerState) {
    energyWasted = HEATER_POWER * deltaTime * 0.001f;
    heatState.thermoHeaterWasted = energyWasted;

    // deduct balance
    heatState.heatBalancePool -= energyWasted;
  }
  return energyWasted;
}


void driveHeaterByEnergyBalance(HeatState& heatState, int timeInterval) {
  float heatBalance = heatState.heatBalancePool;

  uint32_t currentTimestamp = millis();
  int deltaTime = currentTimestamp - heatState.lastBoilerStateTimestamp;
  if (deltaTime > timeInterval) {
    if (heatBalance <= 0) {
      setBoilerOff();
      heatState.lastBoilerStateTimestamp = millis();
      heatState.lastBoilerState = false;
    }
    else {
      setBoilerOn();
      heatState.lastBoilerStateTimestamp = millis();
      heatState.lastBoilerState = true;
    }
  }

}
