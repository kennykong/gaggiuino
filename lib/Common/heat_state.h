/* 14:32 27/12/2023 - change triggering comment */
#ifndef HEAT_STATE_H
#define HEAT_STATE_H


struct HeatState {

  bool brewActive;                      // Only when brewing, thermoCompensateHeat is caculated, otherwize only uses pidAdjustHeat
  bool lastBoilerState;                 // false is OFF, true is ON
  uint32_t lastBoilerStateTimestamp;    // mills
  float lastWaterPumped;                // ml or g
  uint32_t lastWaterPumpedTimestamp;    // mills
  float heatBalancePool;                // Unit is Joule == Watt*Second
  float pidAdjustHeat;                  // PID heat caculate, Unit is Joule
  uint32_t lastPidAdjustTimestamp;      // mills
  double pidOutput;                      // pid out, how many milis boiler on in 1 second
  float thermoCompensateHeat;           // Thermo Compensation heat caculate, Unit is Joule
  float thermoHeaterWasted;             // Unit is Joule
  
};

#endif
