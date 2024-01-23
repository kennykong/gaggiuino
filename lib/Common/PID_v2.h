#ifndef PID_v2_h
#define PID_v2_h
#define LIBRARY_VERSION	2.0.1

class PID
{


  public:

  //Constants used in some of the functions below
  // #define AUTOMATIC	1
  // #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  #define P_ON_M 0    // Proportional on Measurement
  #define P_ON_E 1    // Proportional on Error (default)

  //commonly used functions **************************************************************************
    PID();
    PID(double Kp, double Ki, double Kd, int PMode, int ControllerDirection);   
                                          // * constructor.  links the PID to the Input, Output, and 
                                          //   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    PID(double Kp, double Ki, double Kd);    // * constructor.  links the PID to the Input, Output, and 
                                             //   Setpoint.  Initial tuning parameters are also set here
                                             
    PID(double Kp, double Ki, double Kd, int ControllerDirection); 
	
    // void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    double Compute(double currentValue, 
                  double setPoint);        // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(double Min, double Max);   
                                          // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(double Kp, double Ki, double Kd);         	  
                                          // * While most users will set the tunings once in the 
                                          //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(double Kp, double Ki, double Kd, int PMode);         	  
                                          // * overload for specifying proportional mode

	  void SetControllerDirection(int Direction);	  
                                        // * Sets the Direction, or "Action" of the controller. DIRECT
										                    //   means the output will increase when error is positive. REVERSE
										                    //   means the opposite.  it's very unlikely that this will be needed
										                    //   once it is set in the constructor.
    void SetSampleTime(int NewSampleTime);            
                                        // * sets the frequency, in Milliseconds, with which 
                                        //   the PID calculation is performed.  default is 100
										  
										  
										  
  //Display functions ****************************************************************
	double GetKp();						  // These functions query the pid for interal values.
	double GetKi();						  //  they were created mainly for the pid front-end,
	double GetKd();						  // where it's important to know what is actually 
	// int GetMode();						  //  inside the PID.
	int GetDirection();					//
  unsigned long windowStartTime;

  double GetOutMax();
  double GetOutMin();

  unsigned long GetSampleTime();

  bool IsPOnE();
  void Initialize();

  double regulation(double value, double min, double max);  //regulation a value

  private:
	
	
	double dispKp;				        // * we'll hold on to the tuning parameters in user-entered 
	double dispKi;				        //   format for display purposes
	double dispKd;				        //
    
	double kp;                   // * (P)roportional Tuning Parameter
  double ki;                   // * (I)ntegral Tuning Parameter
  double kd;                   // * (D)erivative Tuning Parameter

	int controllerDirection;
  int pMode;                     // * Proportional on Error (default) OR Proportional on Measurement

  // double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
  // double *myOutput;             //   This creates a hard link between the variables and the 
  // double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.

  	  
	unsigned long lastTime;           // * pid state	
	double outputSum, lastInput;

	unsigned long SampleTime;         // * millis
	double outMin, outMax;
	bool pOnE;                        // * Proportional on Error (default) OR Proportional on Measurement
  bool isFirstRun;                  // * mark First Run
  
};
#endif

