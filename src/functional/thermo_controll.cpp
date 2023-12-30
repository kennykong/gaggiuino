/* 21:26 27/12/2023 - change triggering comment */
/* PID control and thermo compensation*/

#include "thermo_controll.h"

namespace {
  class PIDGroupSingleton {

  public:
    static PID& getOnBrewPID() {
      static PID onBrewPID;
      return onBrewPID;
    };
    static PID& getOffBrewPID() {
      static PID offBrewPID;
      return offBrewPID;
    };
    // static PID& offBrewPID;
    // static PID& onBrewPID;
  private:
    
    PIDGroupSingleton() = default;
    ~PIDGroupSingleton() = default;
  };

}

void myPIDsInit() {
  initOnBrewPID();
  initOffBrewPID();
}


void initOnBrewPID() {

  float dt = HEAT_BREW_TIME_INTERVAL;
  float min = 0.f;
  float max = PID_TIME_WINDOW_SIZE;
  float Kp = 0.5f;
  float Ki = 0.f;
  float Kd = 0.f;

  PIDGroupSingleton::getOnBrewPID().SetTunings(Kp, Ki, Kd);
  PIDGroupSingleton::getOnBrewPID().SetOutputLimits(min, max);
  PIDGroupSingleton::getOnBrewPID().SetSampleTime(dt);
}

void initOffBrewPID() {
  float dt1 = HEAT_TIME_INTERVAL;
  float min1 = 0.f;
  float max1 = PID_TIME_WINDOW_SIZE;
  float Kp1 = 0.f;
  float Ki1 = 0.f;
  float Kd1 = 0.f;

  PIDGroupSingleton::getOffBrewPID().SetTunings(Kp1, Ki1, Kd1);
  PIDGroupSingleton::getOffBrewPID().SetOutputLimits(min1, max1);
  PIDGroupSingleton::getOffBrewPID().SetSampleTime(dt1);
}



PID& getOnBrewPID() {
  return PIDGroupSingleton::getOnBrewPID();
}

PID& getOffBrewPID() {
  return PIDGroupSingleton::getOffBrewPID();
}

float computeThermoCompensateEnergy(float coldWaterTemp, float targetTemp, const SensorState& currentState, HeatState& heatState, int timeInterval) {

  if (coldWaterTemp < 20.f) {
    coldWaterTemp = DEFAULT_GUARANTEE_TEMP_BREW;
  }
  float deltaHeat = 0.f;
  float deltaTemp = targetTemp - coldWaterTemp;
  float currentWaterPumped = currentState.waterPumped;
  float deltaWater = currentWaterPumped - heatState.lastWaterPumped;
  uint32_t currentWaterPumpedTimestamp = millis();
  int deltaTime = currentWaterPumpedTimestamp - heatState.lastWaterPumpedTimestamp;
  if (deltaTime >= timeInterval && deltaWater > 0) {
    heatState.lastWaterPumped = currentWaterPumped;
    heatState.lastWaterPumpedTimestamp = currentWaterPumpedTimestamp;
    float deltaHeat = deltaWater * deltaTemp * WATER_TEMP_RISE_POWER;
    heatState.thermoCompensateHeat = deltaHeat;

    // add balance
    heatState.heatBalancePool += deltaHeat;
  }

  return deltaHeat;
}

// it's realtime job, cant't accumulate by time.
float doPIDAdjust(float targetTemp, PID& pid, const SensorState& currentState, HeatState& heatState, int timeInterval) {
  //reset the heat balance 
  heatState.heatBalancePool = 0.f;
  //do pid
  float output = 0.f;
  float currentTemp = currentState.temperature;
  // uint32_t currentTimestamp = millis();
  // int deltaTime = currentTimestamp - heatState.lastPidAdjustTimestamp;
  // if (deltaTime >= timeInterval) {
    output = pid.Compute(currentTemp, targetTemp);
    heatState.lastPidAdjustTimestamp = millis();
    if (millis() - pid.windowStartTime > PID_TIME_WINDOW_SIZE)
    { //time to shift the Relay Window
      pid.windowStartTime +=  PID_TIME_WINDOW_SIZE;
    }
    if (output <= millis() - pid.windowStartTime) turnOnBoiler(heatState);
    else turnOffBoiler(heatState);
  // }
  return output;
}


float computeHeaterWastedEnergy(HeatState& heatState, int timeInterval) {
  float energyWasted = 0.f;
  uint32_t currentTimestamp = millis();

  int deltaTime = currentTimestamp - heatState.lastBoilerStateTimestamp;
  if (deltaTime >= timeInterval && heatState.lastBoilerState) {
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
  if (deltaTime >= timeInterval) {
    if (heatBalance <= 0) {
      turnOffBoiler(heatState);
    }
    else {
      turnOnBoiler(heatState);
    }
  }
}

void turnOnBoiler(HeatState& heatState){
  setBoilerOn();
  heatState.lastBoilerState = true;
  heatState.lastBoilerStateTimestamp = millis();
}

void turnOffBoiler(HeatState& heatState){
  setBoilerOff();
  heatState.lastBoilerState = false;
  heatState.lastBoilerStateTimestamp = millis();
}

