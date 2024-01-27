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
    static PID& getSteamPID() {
      static PID steamPID;
      return steamPID;
    };
  private:
    
    PIDGroupSingleton() = default;
    ~PIDGroupSingleton() = default;
  };

}

void myPIDsInit() {
  initOnBrewPID();
  initOffBrewPID();
  initSteamPID();
}


void initOnBrewPID() {
  //fast, balanced PID
  int dt = HEAT_BREW_TIME_INTERVAL;
  float min = 0.f;
  float max = MAX_BOILER_ON_PERCENTAGE;

  //Ziegler–Nichols method PI
  //Tu = 31s
  double Kp = 30.f;
  double Ki = 1.161290322580645;
  // double Ki = (1.2 * Kp) / 31.0;
  double Kd = 0.0;

  PIDGroupSingleton::getOnBrewPID().SetTunings(Kp, Ki, Kd);
  PIDGroupSingleton::getOnBrewPID().SetOutputLimits(min, max);
  PIDGroupSingleton::getOnBrewPID().SetSampleTime(dt);
  PIDGroupSingleton::getOnBrewPID().SetId(1);  //onbrew pid is 1
}

void initOffBrewPID() {
  //slow, balanced PID
  int dt1 = HEAT_TIME_INTERVAL;
  float min1 = 0.f;
  float max1 = MAX_BOILER_ON_PERCENTAGE;

  //Ziegler–Nichols method PI
  //Tu = 31s
  double Kp1 = 4.5;
  double Ki1 = 0.1741935483870968;
  // double Ki1 = (1.2 * Kp1) / 31.0;
  double Kd1 = 0.0;

  PIDGroupSingleton::getOffBrewPID().SetTunings(Kp1, Ki1, Kd1);
  PIDGroupSingleton::getOffBrewPID().SetOutputLimits(min1, max1);
  PIDGroupSingleton::getOffBrewPID().SetSampleTime(dt1);
  PIDGroupSingleton::getOffBrewPID().SetId(2);  //offbrew pid is 2
}

void initSteamPID() {
  //fast, aggressvie PID
  int dt = HEAT_BREW_TIME_INTERVAL;
  float min = 0.f;
  float max = MAX_BOILER_ON_PERCENTAGE;

  //Ziegler–Nichols method PI
  //Tu = 31s
  double Kp = 30.f;
  double Ki = 1.2;
  // double Ki = (1.2 * Kp) / 30.0;
  double Kd = 0.0;

  PIDGroupSingleton::getSteamPID().SetTunings(Kp, Ki, Kd);
  PIDGroupSingleton::getSteamPID().SetOutputLimits(min, max);
  PIDGroupSingleton::getSteamPID().SetSampleTime(dt);
  PIDGroupSingleton::getSteamPID().SetId(3);  //steam pid is 3
}

void resetThemoCompState(HeatState& heatState, const SensorState& currentState) {
  
  // reset ThemoComp HeatState
  heatState.heatBalancePool = 0.0;
  heatState.lastWaterPumped = 0.f;
  heatState.lastWaterPumpedTimestamp = millis();
  heatState.lastThermoCompensateHeat = 0.f;
  heatState.lastThermoHeaterConsumed = 0.f;
  heatState.lastTemperature = currentState.temperature;
  heatState.lastTemperatureTime = millis();
  // turnOffBoiler(heatState);  // not necessary ?

  // reset pid status with current state
  int lastPidOperator = heatState.lastPidOperator;
  if (lastPidOperator == 1 || lastPidOperator == 2) {
    double outputSum = heatState.pidOutput;
    double lastInput = currentState.temperature;
    getOnBrewPID().reset(outputSum, lastInput);
    getOffBrewPID().reset(outputSum, lastInput);
  }

}


PID& getOnBrewPID() {
  return PIDGroupSingleton::getOnBrewPID();
}

PID& getOffBrewPID() {
  return PIDGroupSingleton::getOffBrewPID();
}

PID& getSteamPID() {
  return PIDGroupSingleton::getSteamPID();
}

float computeThermoCompensateEnergyByInletWater(float compensateTemp, float targetTemp, const SensorState& currentState, HeatState& heatState, int timeInterval) {

  float deltaHeat = 0.f;
  float currentWaterPumped = currentState.waterPumped;
  float deltaWater = currentWaterPumped - heatState.lastWaterPumped;

  uint32_t currentWaterPumpedTimestamp = millis();
  uint32_t lastWaterPumpedTimestamp = heatState.lastWaterPumpedTimestamp;
  //timer overflow
  if (currentWaterPumpedTimestamp < lastWaterPumpedTimestamp) {
    heatState.lastWaterPumpedTimestamp = currentWaterPumpedTimestamp;
    return 0.f;
  }

  int deltaTime = currentWaterPumpedTimestamp - lastWaterPumpedTimestamp;
  
  if (deltaWater > 0 && deltaTime >= timeInterval) {
    heatState.lastWaterPumped = currentWaterPumped;
    heatState.lastWaterPumpedTimestamp = currentWaterPumpedTimestamp;

    // float deltaTemp = targetTemp - inletWaterTemp;
    // float currentTemp = currentState.temperature;
    
    // float correctionTemp = caculateCorrection(heatState, currentTemp, targetTemp);
    // float correctionTemp = 0.f;
    float deltaHeat = deltaWater * compensateTemp * WATER_TEMP_RISE_POWER;
    heatState.lastThermoCompensateHeat = deltaHeat;

    // add balance
    heatState.heatBalancePool += deltaHeat;
  }

  return deltaHeat;
}

// Unused yet
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
        else if (lastTempOffset > 0 && currentTempOffset < 0) {
          correctionTemp = -(lastTempOffset + currentTempOffset);
        }
        else if (lastTempOffset < 0 && currentTempOffset > 0) {
          correctionTemp = -(lastTempOffset + currentTempOffset);
        }
        else if (lastTempOffset < 0 && currentTempOffset < 0) {
          correctionTemp = lastTempOffset - currentTempOffset;
        }
      }
      
    }
    heatState.lastTemperature = currentTemp;
    heatState.lastTemperatureTime = currentTime;
  }
  return correctionTemp;
}

float computeHeaterConsumedEnergyAndDoHeat(HeatState& heatState, float currentTemp, float setPoint, float upperLimit, float downLimit, int timeInterval) {
  float energyConsumed = 0.f;
  bool isMyOperation = heatState.isBoilerOperatorTC;
  uint32_t lastHeaterWaveTime = heatState.lastBoilerStateTimestamp;
  uint32_t currentTime = micros();
  //timer overflow
  if (currentTime < lastHeaterWaveTime) {
    heatState.lastBoilerStateTimestamp = currentTime;
    return 0.f;
  }
  unsigned long deltaTime = currentTime - lastHeaterWaveTime;
  if (deltaTime >= timeInterval * 1000UL) {
    if (isMyOperation && heatState.lastBoilerState) {

      energyConsumed = HEATER_POWER * deltaTime * 0.000001f;
      heatState.lastThermoHeaterConsumed = energyConsumed;

      // reduce balance
      heatState.heatBalancePool -= energyConsumed;
    }
    // drive Heater
    driveHeaterByEnergyBalance(heatState, currentTemp, setPoint, upperLimit, downLimit);
  }
  
  return energyConsumed;
}

void driveHeaterByEnergyBalance(HeatState& heatState, float currentTemp, float setPoint, float upperLimit, float downLimit) {
  bool boilerOperatorTC = true;
  if (heatState.heatBalancePool <= 0.f) {
    if (currentTemp >= setPoint - downLimit) {
      turnOffBoiler(heatState, boilerOperatorTC);
    }
    else {
      turnOnBoiler(heatState, boilerOperatorTC);
    }
  }
  else {
    if (currentTemp <= setPoint + upperLimit) {
      turnOnBoiler(heatState, boilerOperatorTC);
    }
    else {
      turnOffBoiler(heatState, boilerOperatorTC);
    }
  }
}

// it's realtime job, cant't accumulate by time.
double doPIDAdjust(float targetTemp, PID& pid, const SensorState& currentState, HeatState& heatState) {
  
  //do pid
  double outMin = pid.GetOutMin();
   //compute output
  double output = pid.Compute(currentState.temperature, targetTemp);
  //only if output is legal, update the heatState's output to new value
  if (output >= outMin) {
    heatState.lastPidOutputTimestamp = micros();
    heatState.lastPidOperator = pid.GetId();
    heatState.pidOutput = output;
  }
  //
  //pid turn 10 times in one powerCycle, so powerCycle = sampleTime *10
  //sampleTime from ms to us *1000
  //so *10000UL
  unsigned long powerAdjustCycle = pid.GetSampleTime() * 10000UL;
  //heat boiler use the latest output
  pulseHeaters(heatState, powerAdjustCycle);
  return output;
}

double doPIDAdjustWithLimit(float targetTemp, float downLimit, float upperLimit, PID& pidController, const SensorState& currentState, HeatState& heatState) {
  float currentTemp = currentState.temperature;
  double out = pidController.GetOutMin() - 2.0;
  if (currentTemp < targetTemp - downLimit) {
    turnOnBoiler(heatState);
  }
  else if (currentTemp > targetTemp + upperLimit) {
    turnOffBoiler(heatState);
  }
  else {
    out = doPIDAdjust(targetTemp, pidController, currentState, heatState);
  }
  return out;
}

void pulseHeaters(HeatState& heatState, unsigned long powerAdjustCycle) {
  uint32_t lastHeaterWaveTime = heatState.lastBoilerStateTimestamp;
  bool boilerOn = heatState.lastBoilerState;
  // output%, so * 0.01
  // powerAdjustCycle is in micro seconds
  unsigned long onTime = round(heatState.pidOutput * 0.01 * powerAdjustCycle);
  uint32_t currentTime = micros();
  //timer overflow
  if (currentTime < lastHeaterWaveTime) {
    heatState.lastBoilerStateTimestamp = currentTime;
    return;
  }
  uint32_t deltaTime = currentTime - lastHeaterWaveTime;
  if (!boilerOn && (deltaTime >= (powerAdjustCycle - onTime))) {
    turnOnBoiler(heatState);
  }
  else if (boilerOn && (deltaTime >= onTime)) {
    turnOffBoiler(heatState);
  }
}


void turnOnBoiler(HeatState& heatState, bool isBoilerOperatorTC){
  setBoilerOn();
  heatState.lastBoilerState = true;
  heatState.lastBoilerStateTimestamp = micros();
  heatState.isBoilerOperatorTC = isBoilerOperatorTC;
}

void turnOffBoiler(HeatState& heatState, bool isBoilerOperatorTC){
  setBoilerOff();
  heatState.lastBoilerState = false;
  heatState.lastBoilerStateTimestamp = micros();
  heatState.isBoilerOperatorTC = isBoilerOperatorTC;
}

void turnOnBoiler(HeatState& heatState) {
  turnOnBoiler(heatState, false);
}

void turnOffBoiler(HeatState& heatState) {
  turnOffBoiler(heatState, false);
}

