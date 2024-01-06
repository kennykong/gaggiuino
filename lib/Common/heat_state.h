/* 14:32 27/12/2023 - change triggering comment */
#ifndef HEAT_STATE_H
#define HEAT_STATE_H


struct HeatState {

  bool brewActive;                          // Only when brewing, thermoCompensateHeat is caculated, otherwize only uses pidAdjustHeat
  bool lastBoilerState;                     // false is OFF, true is ON
  uint32_t lastBoilerStateTimestamp;        // mills
  float lastWaterPumped;                    // ml or g
  uint32_t lastWaterPumpedTimestamp;        // mills
  float heatBalancePool;                    // Unit is Joule == Watt*Second
  uint32_t lastPidOutputTimestamp;          // mills
  float pidOutput;                          // pid out, how many milis boiler on in 1 second, no need to reset when brew on-off
  float lastTemperature;                    // last Sensor read temperature
  float lastTemperatureTime;                // last Sensor read temperature timw stamp
  float temperatureRiseSpeed; 
  float lastThermoCompensateHeat;           // Thermo Compensation heat caculate, Unit is Joule
  float lastThermoHeaterConsumed;           // Unit is Joule
  bool isBoilerOperatorTC;                  // Who operated the boiler, true is Thermo Compensation, false is PID or Hard Limit.

};

#endif
