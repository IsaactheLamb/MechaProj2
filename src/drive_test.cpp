#include <math.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fstream>

#include <wiringPi.h>
#include <softPwm.h>

#include "MiniPID/MiniPID.h"

using namespace std;

#define PWM_PIN_L1   6
#define PWM_PIN_L2   5
#define PWM_PIN_R1   11
#define PWM_PIN_R2   10

#define ENC_PIN_L1   16 
#define ENC_PIN_L2   15
#define ENC_PIN_R1   4
#define ENC_PIN_R2   1

#define ENC_HZ 50

#define MAX_PWM 4096
#define MTR_MAX_RPM 85
#define DES_MAX_RPM 30

static volatile int enc_cnt_L = 0;
static volatile int enc_cnt_R = 0;

static volatile float rpm_L = 0;
static volatile float rpm_R = 0;

// Desired RPM values for each wheel
static volatile float des_rpm_L = 0.0; 
static volatile float des_rpm_R = 0.0; 

// Angle tracking
float angle = 0;
float dontcare = 0;

void encISR_L() 
{
  ++enc_cnt_L;
}

void encISR_R() 
{
  ++enc_cnt_R;
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

PI_THREAD (imuThread)
{  
  while(1)
  {
	  scanf("%f %f", &angle, &dontcare);

	  cout << "Angle is " << angle << endl;

	  // Read to end of line so don't get stuck on one invalid line.
	  while(getc(stdin) != '\n');
  }
}

int main(int argc, char **argv)
{
  wiringPiSetup();
  piThreadCreate (encThread); // Start encoder thread
  piThreadCreate (imuThread); // Start IMU thread

  double c1_p = 0.02;
  double c1_i = 0;
  double c1_d = 0.05;

  if (argc == 4)
  {
    c1_p = stod(argv[1]);
    c1_i = stod(argv[2]);
    c1_d = stod(argv[3]);
  }

  MiniPID pid_L = MiniPID(c1_p, c1_i, c1_d);
  MiniPID pid_R = MiniPID(c1_p, c1_i, c1_d);

  float pwm_val_L = 0; // PWM values to write to pins
  float pwm_val_R = 0; 

  bool dir_L = 0; // Wheel directions
  bool dir_R = 0;

  // pin: GPIO 14, initial val: 0, pwm range: 100 (treating as %)  
  softPwmCreate (PWM_PIN_L1, 0, MAX_PWM);
  softPwmCreate (PWM_PIN_L2, 0, MAX_PWM);
  softPwmCreate (PWM_PIN_R1, 0, MAX_PWM);
  softPwmCreate (PWM_PIN_R2, 0, MAX_PWM);



    // PID tuning
  double c2_p = 0.2;
  double c2_i = 0;
  double c2_d = 0.05;

  MiniPID pid_IMU = MiniPID(c2_p, c2_i, c2_d);
  pid_IMU.setOutputLimits(0, 0.2);

  for(;;) 
  {
    // *********************** IMU PID CONTROL ***********************
    // -1 is death, 1 is death
    // Try keeping between -0.2 and 0.2
    double out_IMU = pid_IMU.getOutput(angle, 0.0); 

    // Multiply by 5 to make unit value at max of 0.2
    des_rpm_L += MAX_PWM*out_IMU /DES_MAX_RPM;
    des_rpm_R += MAX_PWM*out_IMU /DES_MAX_RPM;

    cout << "Desired RPM LEFT IS          : " << des_rpm_L << endl;

    // ******************** DRIVE WHEEL PID CONTROL ********************
    double out_L = pid_L.getOutput(rpm_L, des_rpm_L); // Get delta in PWM
    double out_R = pid_R.getOutput(rpm_R, des_rpm_R); 
    //cout << "PID Output: " << output << endl; 	

    pwm_val_L += MAX_PWM*out_L/MTR_MAX_RPM; // Increment PWM
    pwm_val_R += MAX_PWM*out_R/MTR_MAX_RPM; 

    // Set directions
    if (pwm_val_L < 0) dir_L = 1;
    else dir_L = 0;

    if (pwm_val_R < 0) dir_R = 1;
    else dir_R = 0;

    // Constrain PWM values
    if (pwm_val_L > MAX_PWM/10) pwm_val_L = MAX_PWM/10;
    else if (pwm_val_L < -MAX_PWM/10) pwm_val_L = -MAX_PWM/10;

    if (pwm_val_R > MAX_PWM/10) pwm_val_R = MAX_PWM/10;
    else if (pwm_val_R < -MAX_PWM/10) pwm_val_R = -MAX_PWM/10;

    if (des_rpm_L > DES_MAX_RPM) des_rpm_L = DES_MAX_RPM;
    else if (des_rpm_L < -DES_MAX_RPM) des_rpm_L = -DES_MAX_RPM;

    if (des_rpm_R > DES_MAX_RPM) des_rpm_R = DES_MAX_RPM;
    else if (des_rpm_R < -DES_MAX_RPM) des_rpm_R = -DES_MAX_RPM;

	cout << "pwm PID           " << out_L << endl;
	cout << "pwm actual " << pwm_val_L << endl;
	cout << "pwm commanded       " << dir_L*fabs(pwm_val_L) << endl;
	cout << "wheel dir L    :  " << dir_L << endl;

    // Set PWMs
    softPwmWrite(PWM_PIN_L1, (int) dir_L*fabs(pwm_val_L)); 
    softPwmWrite(PWM_PIN_L2, (int) (!dir_L)*fabs(pwm_val_L));
    softPwmWrite(PWM_PIN_R1, (int) dir_R*fabs(pwm_val_R)); 
    softPwmWrite(PWM_PIN_R2, (int) (!dir_R)*fabs(pwm_val_R));

    delay(1000.0/ENC_HZ); // Delay to let PID code work its magic

  }
}
