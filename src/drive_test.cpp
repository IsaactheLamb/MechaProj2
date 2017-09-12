#include <math.h>
#include <stdio.h>
#include <iostream>

#include <wiringPi.h>
#include <softPwm.h>

#include "MiniPID/MiniPID.h"

using namespace std;

#define PWM_PIN_L1   15
#define PWM_PIN_L2   -
#define PWM_PIN_R1   -
#define PWM_PIN_R2   -

#define ENC_PIN_L1   16
#define ENC_PIN_L2   1
#define ENC_PIN_R1   -
#define ENC_PIN_R2   -

#define ENC_HZ 50

#define MAX_PWM 4096
#define MTR_MAX_RPM 85

static volatile int enc_cnt_L = 0;
static volatile int enc_cnt_R = 0;
static volatile float rpm_L = 0;
static volatile float rpm_R = 0;

void encISR_L() 
{
  ++enc_count_L;
}

void encISR_R() 
{
  ++enc_count_R;
}

PI_THREAD (encThread)
{
  // Set up encoder pins for interrupts
  wiringPiISR(ENC_PIN_L1, INT_EDGE_RISING, &encISR_L);
  wiringPiISR(ENC_PIN_L2, INT_EDGE_RISING, &encISR_L);
  wiringPiISR(ENC_PIN_R1, INT_EDGE_RISING, &encISR_R);
  wiringPiISR(ENC_PIN_R2, INT_EDGE_RISING, &encISR_R);

  float del_time = 1.0/((float)ENC_HZ);
  float ppr = 150*6; // Pulses per revolution at output shaft
  
  for (;;)
  {
    // Reset encoder counters
    enc_cnt_L = 0;
    enc_cnt_R = 0;

    delay(1000.0*del_time); // Delay in ms between readings

    // Calculate RPM of each wheel
    rpm_L = 60.0*((enc_cnt_L/ppr)/del_time);
    rpm_R = 60.0*((enc_cnt_R/ppr)/del_time);

    //printf("Angular velocity: %.2f rad/s.", ang_vel);
    cout << "RPM Left  :" << (int) rpm_L << endl;
    cout << "RPM Right :" << (int) rpm_R << endl;
  }
}

int main(int argc, char **argv)
{
  wiringPiSetup();
  piThreadCreate (encThread); // Start encoder thread

  double c_p = 0.02;
  double c_i = 0;
  double c_d = 0.05;

  if (argc == 4)
  {
    c_p = stod(argv[1]);
    c_i = stod(argv[2]);
    c_d = stod(argv[3]);
  }

  MiniPID pid_L = MiniPID(c_p, c_i, c_d);
  MiniPID pid_R = MiniPID(c_p, c_i, c_d);

  float des_rpm_L = 20.0; // Desired RPM values for each wheel
  float des_rpm_R = 20.0; 

  float pwm_val_L = 0; // PWM values to write to pins
  float pwm_val_R = 0; 

  bool dir_L = 0; // Wheel directions
  bool dir_R = 0;

  // pin: GPIO 14, initial val: 0, pwm range: 100 (treating as %)  
  softPwmCreate (PWM_PIN_L1, 0, MAX_PWM);
  softPwmCreate (PWM_PIN_L2, 0, MAX_PWM);
  softPwmCreate (PWM_PIN_R1, 0, MAX_PWM);
  softPwmCreate (PWM_PIN_R2, 0, MAX_PWM);

  for(;;) 
  {
    double out_L = pid_L.getOutput(rpm_L, des_rpm_L); // Get delta in PWM
    double out_R = pid_R.getOutput(rpm_R, des_rpm_R); 
    //cout << "PID Output: " << output << endl; 	

    pwm_val_L += (out_L/MTR_MAX_RPM)*MAX_PWM; // Increment PWM
    pwm_val_R += (out_R/MTR_MAX_RPM)*MAX_PWM; 

    // Set directions
    if (pwm_val_L < 0) dir_L = 1;
    else dir_L = 0;

    if (pwm_val_R < 0) dir_R = 1;
    else dir_R = 0;

    // Constrain PWM values
    if (pwm_val_L > MAX_PWM) pwm_val_L = MAX_PWM;
    else if (pwm_val_L < -MAX_PWM) pwm_val_L = -MAX_PWM;

    if (pwm_val_L > MAX_PWM) pwm_val_L = MAX_PWM;
    else if (pwm_val_L < -MAX_PWM) pwm_val_L = -MAX_PWM;

    // Set PWMs
    softPwmWrite(PWM_PIN_L1, (int) dir_L*fabs(pwm_val_L)); 
    softPwmWrite(PWM_PIN_L2, (int) (!dir_L)*fabs(pwm_val_L));
    softPwmWrite(PWM_PIN_R1, (int) dir_R*fabs(pwm_val_R)); 
    softPwmWrite(PWM_PIN_R2, (int) (!dir_R)*fabs(pwm_val_R));

    //cout << "PWM val: " << pwm_val << endl;

    delay(1000.0/ENC_HZ); // Delay to let PID code work its magic

  }
}

/*
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <iostream>
using namespace std;

while(1)
	    {
		float angle = 0;
		float dontcare = 0;

		int result = scanf("%f %f",
		                   &angle, &dontcare);

		cout << "Angle is " << angle << endl;

		// Read to the end of the line so that we don't get stuck forever on one invalid line.
		while(getc(stdin) != '\n');

		if (result >= 9)
		{
		    break; // Success
		}
	    }
*/
