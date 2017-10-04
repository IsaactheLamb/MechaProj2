#include <math.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fstream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

 #include <signal.h> 
#include <sys/types.h>

#include <wiringPi.h>
#include <softPwm.h>

#include "MiniPID/MiniPID.h"

using namespace cv;
using namespace std;

#define PWM_PIN_L1   6
#define PWM_PIN_L2   5
#define PWM_PIN_R1   11
#define PWM_PIN_R2   10

#define ENC_PIN_L1   0
#define ENC_PIN_L2   7
#define ENC_PIN_R1   4
#define ENC_PIN_R2   1

#define ENC_HZ 50

#define MAX_PWM 256
#define MTR_MAX_RPM 110

static volatile int enc_cnt_L = 0;
static volatile int enc_cnt_R = 0;

static volatile float rpm_L = 0;
static volatile float rpm_R = 0;

static volatile bool dir_L = 0; // Wheel directions
static volatile bool dir_R = 0;

// Desired RPM values for each wheel
static volatile float des_rpm = 0.0; 

// Angle tracking
static volatile float angle = 0;

const float loop_del = 1000.0/ENC_HZ;

// Function for clamping a value between an upper and lower bound
double clamp(double val, double upper, double lower) 
{
  return = min(upper, max(val, lower));
}

// Function to return sign of a number
int sign(double val) 
{
  return (val > 0) - (val < 0);
}

// Encoder interrupt service routine for left wheel
void encISR_L() {
  ++enc_cnt_L;
}

// Encoder interrupt service routine for right wheel
void encISR_R() {
  ++enc_cnt_R;
}

/*
PI_THREAD (encThread)
{
  // Set up encoder pins for interrupts
  wiringPiISR(ENC_PIN_L1, INT_EDGE_RISING, &encISR_L);
  wiringPiISR(ENC_PIN_L2, INT_EDGE_RISING, &encISR_L);
  wiringPiISR(ENC_PIN_R1, INT_EDGE_RISING, &encISR_R);
  wiringPiISR(ENC_PIN_R2, INT_EDGE_RISING, &encISR_R);

  float del_time = 1.0/((float)ENC_HZ);
  float ppr = 150*6; // Pulses per revolution at output shaft

  float coeff = 60.0/(ppr*del_time); // 2x is for only 2 enc pins

  // Direction of RPM
  int dirs[2] = {-1, 1};
  
  for (;;)
  {
    // Reset encoder counters
    enc_cnt_L = 0;
    enc_cnt_R = 0;

    delay(loop_del); // Delay in ms between readings

    // Calculate RPM of each wheel
    rpm_L = dirs[dir_L]*coeff*enc_cnt_L;
    rpm_R = dirs[dir_R]*coeff*enc_cnt_R;
    
    cout << "\tRPM Left  :" << rpm_L << endl;
    cout << "\t\tRPM Right :" << rpm_R << endl;
  }
}
*/

PI_THREAD (imuThread)
{  
  for(;;)
  {
    // **** CAPTURE ANGLE ***********************************************
    scanf("%f *", &angle);        // Grab angle from stdin
	  angle = (angle + 0.05)*90;    // Add value to calibrate, convert to degrees
    while(getc(stdin) != '\n');   // Read to end of line to not get stuck
  }
}

int main(int argc, char **argv)
{
  wiringPiSetup();              // Initialise wiringPi
  piThreadCreate (imuThread);   // Start IMU angle thread
  //piThreadCreate (encThread); // Start encoder thread

  // Set default PID gains
  double c_p = 0.02;
  double c_i = 0;
  double c_d = 0.05;

  // Grab desired PID gains from command line
  if (argc == 4) {
    c_p = stod(argv[1]);
    c_i = stod(argv[2]);
    c_d = stod(argv[3]);
  }
  
  MiniPID pid = MiniPID(c_p, c_i, c_d); // Initialise the PID controller
  pid.setOutputLimits(MAX_PWM/2);       // Limit PID output to part of PWM range

  double output = 0;   // PID output variable
  double setpoint = 0; // Zero degrees, PID setpoint value

  double deadband = 5;    // PWM values for which we want to clip to zero 
  double sp_incr = 0.05; // Value to increment setpoint by

  float pwm_val_L = 0; // PWM value for left wheel (0 to 100)
  float pwm_val_R = 0; // PWM value for right wheel (0 to 100)

  // Initialise pins for PWM control of wheels
  softPwmCreate (PWM_PIN_L1, 0, MAX_PWM);
  softPwmCreate (PWM_PIN_L2, 0, MAX_PWM);
  softPwmCreate (PWM_PIN_R1, 0, MAX_PWM);
  softPwmCreate (PWM_PIN_R2, 0, MAX_PWM);

  for(;;) 
  {
    // **** PID CONTROL *************************************************
    output = pid.getOutput(angle, setpoint); // Perform PID calculation

    if (abs(output) < deadband) output = 0;  // Clip output to deadband

    pwm_val_L = output; // Increment PWM values according to PID output
    pwm_val_R = output; 
    
    // **** DIRECTION AND BOUNDARIES ************************************
    dir_L = pwm_val_L >= 0; // Set directions
    dir_R = pwm_val_R >= 0;	
	    
    pwm_val_L = clamp(pwm_val_L, MAX_PWM, -MAX_PWM) // Constrain PWM values
    pwm_val_R = clamp(pwm_val_R, MAX_PWM, -MAX_PWM) 

    // If angle is greater than ... degrees, stop motors
    if (abs(angle) > 50) 
	  {
		  pwm_val_L = 0;
		  pwm_val_R = 0;
	  }

    // If we are moving in one direction, adjust setpoint in same direction
    // to compensate.
    setpoint = setpoint + sign(output)*sp_incr; 

    // **** DRIVE THE MOTORS ********************************************
    softPwmWrite(PWM_PIN_L1, (int) dir_L*fabs(pwm_val_L));    // Set PWMs
    softPwmWrite(PWM_PIN_L2, (int) (!dir_L)*fabs(pwm_val_L));
    softPwmWrite(PWM_PIN_R1, (int) dir_R*fabs(pwm_val_R)); 
    softPwmWrite(PWM_PIN_R2, (int) (!dir_R)*fabs(pwm_val_R));

    delay(loop_del); // Delay to let PID code work its magic
  }
}
