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
  float max = MAX_BOILER_ON_TIME;
  float Kp = 300.f;
  float Ki = 12.f;
  float Kd = 0.f;

  PIDGroupSingleton::getOnBrewPID().SetTunings(Kp, Ki, Kd);
  PIDGroupSingleton::getOnBrewPID().SetOutputLimits(min, max);
  PIDGroupSingleton::getOnBrewPID().SetSampleTime(dt);
}

void initOffBrewPID() {
  float dt1 = HEAT_TIME_INTERVAL;
  float min1 = 0.f;
  float max1 = MAX_BOILER_ON_TIME;
  float Kp1 = 45.f;
  float Ki1 = 1.8f;
  float Kd1 = 0.f;

  PIDGroupSingleton::getOffBrewPID().SetTunings(Kp1, Ki1, Kd1);
  PIDGroupSingleton::getOffBrewPID().SetOutputLimits(min1, max1);
  PIDGroupSingleton::getOffBrewPID().SetSampleTime(dt1);
}

void resetThemoCompState(HeatState& heatState, const SensorState& currentState) {
  
  // reset ThemoComp HeatState
  heatState.heatBalancePool = 0.f;
  heatState.lastWaterPumped = 0.f;
  heatState.lastWaterPumpedTimestamp = millis();
  heatState.lastThermoCompensateHeat = 0.f;
  heatState.lastThermoHeaterConsumed = 0.f;
  heatState.lastTemperature = currentState.temperature;
  heatState.lastTemperatureTime = millis();
  // turnOffBoiler(heatState);  // not necessary

}


PID& getOnBrewPID() {
  return PIDGroupSingleton::getOnBrewPID();
}

PID& getOffBrewPID() {
  return PIDGroupSingleton::getOffBrewPID();
}

float computeThermoCompensateEnergyByInletWater(float inletWaterTemp, float targetTemp, const SensorState& currentState, HeatState& heatState, int timeInterval) {

  float deltaHeat = 0.f;
  float currentWaterPumped = currentState.waterPumped;
  float deltaWater = currentWaterPumped - heatState.lastWaterPumped;
  uint32_t currentWaterPumpedTimestamp = millis();
  int deltaTime = currentWaterPumpedTimestamp - heatState.lastWaterPumpedTimestamp;
  if (deltaWater > 0 && deltaTime >= timeInterval) {
    heatState.lastWaterPumped = currentWaterPumped;
    heatState.lastWaterPumpedTimestamp = currentWaterPumpedTimestamp;

    float deltaTemp = targetTemp - inletWaterTemp;
    // float currentTemp = currentState.temperature;
    
    // float correctionTemp = caculateCorrection(heatState, currentTemp, targetTemp);
    // float correctionTemp = 0.f;
    float deltaHeat = deltaWater * deltaTemp * WATER_TEMP_RISE_POWER;
    heatState.lastThermoCompensateHeat = deltaHeat;

    // add balance
    heatState.heatBalancePool += deltaHeat;
  }

  return deltaHeat;
}


float caculateCorrection(HeatState& heatState, float currentTemp, float setPoint) {
  float correctionTemp = 0;
  uint32_t currentTime = millis();

  if (currentTime - heatState.lastTemperatureTime >= TEMP_CHECK_INTERVAL) {

    float lastTemp = heatState.lastTemperature;
    if (abs(currentTemp - lastTemp) >= 0.25f) {
      float lastTempOffset = lastTemp - setPoint;
      float currentTempOffset = currentTemp - setPoint;

      // if not converge
      if (abs(lastTempOffset) - abs(currentTempOffset) < 0) {
        if (lastTempOffset > 0 && currentTempOffset > 0) {
          correctionTemp = lastTempOffset - currentTempOffset;
        }
        if (lastTempOffset > 0 && currentTempOffset < 0) {
          correctionTemp = -(lastTempOffset + currentTempOffset);
        }
        if (lastTempOffset < 0 && currentTempOffset > 0) {
          correctionTemp = -(lastTempOffset + currentTempOffset);
        }
        if (lastTempOffset < 0 && currentTempOffset < 0) {
          correctionTemp = lastTempOffset - currentTempOffset;
        }
      }
      
    }
    heatState.lastTemperature = currentTemp;
    heatState.lastTemperatureTime = currentTime;
  }
  return correctionTemp;
}

float computeHeaterConsumedEnergyAndDoHeat(HeatState& heatState, float currentTemp, float setPoint, int timeInterval) {
  float energyConsumed = 0.f;
  bool isMyOperation = heatState.isBoilerOperatorTC;
  int deltaTime = millis() - heatState.lastBoilerStateTimestamp;
  if (deltaTime >= timeInterval) {
    if (isMyOperation && heatState.lastBoilerState) {

      energyConsumed = HEATER_POWER * deltaTime * 0.001f;
      heatState.lastThermoHeaterConsumed = energyConsumed;

      // reduce balance
      heatState.heatBalancePool -= energyConsumed;
    }
    // drive Heater
    driveHeaterByEnergyBalance(heatState, currentTemp, setPoint);
  }
  
  return energyConsumed;
}

void driveHeaterByEnergyBalance(HeatState& heatState, float currentTemp, float setPoint) {
  float heatBalance = heatState.heatBalancePool;
  bool boilerOperatorTC = true;
  if (heatBalance <= 0) {
    if (currentTemp >= setPoint - 2.5f) {
      turnOffBoiler(heatState, boilerOperatorTC);
    }
    else {
      turnOnBoiler(heatState, boilerOperatorTC);
    }
  }
  else {
    if (currentTemp <= setPoint + 0.5f) {
      turnOnBoiler(heatState, boilerOperatorTC);
    }
    else {
      turnOffBoiler(heatState, boilerOperatorTC);
    }
  }
}

// it's realtime job, cant't accumulate by time.
float doPIDAdjust(float targetTemp, PID& pid, const SensorState& currentState, HeatState& heatState) {
  
  //do pid
  float outMin = pid.GetOutMin();
  //init a illegal output
  float output = outMin -1.f;
  //compute output
  output = pid.Compute(currentState.temperature, targetTemp);
  //only if output is legal, update the heatState's output to new value
  if (output >= outMin) {
    heatState.lastPidOutputTimestamp = millis();
    heatState.pidOutput = output;
  }
  //heat boiler use the latest output
  pulseHeaters(heatState);
  return output;
}

void pulseHeaters(HeatState& heatState) {
  uint32_t heaterWaveTime = heatState.lastBoilerStateTimestamp;
  bool boilerOn = heatState.lastBoilerState;
  unsigned long onTime = heatState.pidOutput;
  uint32_t currentTime = millis();
  if (!boilerOn && ((currentTime - heaterWaveTime) >= (1000UL - onTime))) {
    turnOnBoiler(heatState);
  }
  else if (boilerOn && ((currentTime - heaterWaveTime) >= onTime)) {
    turnOffBoiler(heatState);
  }
}


void turnOnBoiler(HeatState& heatState, bool isBoilerOperatorTC){
  setBoilerOn();
  heatState.lastBoilerState = true;
  heatState.lastBoilerStateTimestamp = millis();
  heatState.isBoilerOperatorTC = isBoilerOperatorTC;
}

void turnOffBoiler(HeatState& heatState, bool isBoilerOperatorTC){
  setBoilerOff();
  heatState.lastBoilerState = false;
  heatState.lastBoilerStateTimestamp = millis();
  heatState.isBoilerOperatorTC = isBoilerOperatorTC;
}

void turnOnBoiler(HeatState& heatState) {
  turnOnBoiler(heatState, false);
}

void turnOffBoiler(HeatState& heatState) {
  turnOffBoiler(heatState, false);
}

