/* 14:32 27/12/2023 - change triggering comment */
#ifndef HEAT_STATE_H
#define HEAT_STATE_H


struct HeatState {

  bool brewActive;                          // Only when brewing, thermoCompensateHeat is caculated, otherwize only uses pidAdjustHeat
  bool lastBoilerState;                     // false is OFF, true is ON
  uint32_t lastBoilerStateTimestamp;        // micros for better accurcy, for better RTD Sensor pt100 & ad7124 ADC.
  float lastWaterPumped;                    // ml or g
  uint32_t lastWaterPumpedTimestamp;        // mills
  float heatBalancePool;                    // Unit is Joule == Watt*Second
  uint32_t lastPidOutputTimestamp;          // micros for better accurcy
  double pidOutput;                         // pid out, percentage of full power, no need to reset when brew on-off
  float lastTemperature;                    // last Sensor read temperature
  uint32_t lastTemperatureTime;             // last Sensor read temperature timestamp
  float lastThermoCompensateHeat;           // Thermo Compensation heat caculate, Unit is Joule
  float lastThermoHeaterConsumed;           // Unit is Joule
  bool isBoilerOperatorTC;                  // Who operated the boiler, true is Thermo Compensation, false is PID or Hard Limit.

};

#endif
