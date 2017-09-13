#include <math.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fstream>

 #include <signal.h> 
#include <sys/types.h>

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

#define MAX_PWM 100
#define MTR_MAX_RPM 85

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
const float out_2_in = ((float) MAX_PWM)/MTR_MAX_RPM;

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

    //printf("Angular velocity: %.2f rad/s.", ang_vel);
    cout << "RPM Left  :" << (int) rpm_L << endl;
    cout << "RPM Right :" << (int) rpm_R << endl;
  }
}

PI_THREAD (imuThread)
{  
  while(1)
  {
	  scanf("%f *", &angle);
	  angle = angle - 0.025;

	  cout << "Angle is " << angle << endl;

	  // Read to end of line so don't get stuck on one invalid line.
	  while(getc(stdin) != '\n');

    //delay(loop_del);
  }
}

PI_THREAD (imuThread)
{  


}

static void myExit(void)
{
    softPwmWrite(PWM_PIN_L1, 0); 
    softPwmWrite(PWM_PIN_L2, 0);
    softPwmWrite(PWM_PIN_R1, 0); 
    softPwmWrite(PWM_PIN_R2, 0);

}

int main(int argc, char **argv)
{
  wiringPiSetup();
  piThreadCreate (encThread); // Start encoder thread
  piThreadCreate (imuThread); // Start IMU thread

  atexit(myExit);

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

  // pin: GPIO 14, initial val: 0, pwm range: 100 (treating as %)  
  softPwmCreate (PWM_PIN_L1, 0, MAX_PWM);
  softPwmCreate (PWM_PIN_L2, 0, MAX_PWM);
  softPwmCreate (PWM_PIN_R1, 0, MAX_PWM);
  softPwmCreate (PWM_PIN_R2, 0, MAX_PWM);

  double out = 0; // PID output

  for(;;) 
  {
    // *********************** IMU CONTROL ***********************
    des_rpm = angle*40.0;
    cout << "desired RPM is " << des_rpm << endl;

    // ******************** DRIVE WHEEL PID CONTROL ********************
    double out_L = pid_L.getOutput(rpm_L, des_rpm); // Get delta in PWM
    double out_R = pid_R.getOutput(rpm_R, des_rpm); // Get delta in PWM

    pwm_val_L += out_2_in*out_L; // Increment PWM
    pwm_val_R += out_2_in*out_R; 

    // Set directions
    dir_L = pwm_val_L >= 0;
    dir_R = pwm_val_R >= 0;

	cout << "Dir Left is " << dir_L << endl;
	cout << "Dir right is " << dir_R << endl;

    // Constrain PWM values
    if (pwm_val_L > MAX_PWM) pwm_val_L = MAX_PWM;
    else if (pwm_val_L < -MAX_PWM) pwm_val_L = -MAX_PWM;

    if (pwm_val_R > MAX_PWM) pwm_val_R = MAX_PWM;
    else if (pwm_val_R < -MAX_PWM) pwm_val_R = -MAX_PWM;

    cout << "WRITING TO PWMS *******************************************************" << endl;

    // Set PWMs
    softPwmWrite(PWM_PIN_L1, (int) dir_L*fabs(pwm_val_L)); 
    softPwmWrite(PWM_PIN_L2, (int) (!dir_L)*fabs(pwm_val_L));
    softPwmWrite(PWM_PIN_R1, (int) dir_R*fabs(pwm_val_R)); 
    softPwmWrite(PWM_PIN_R2, (int) (!dir_R)*fabs(pwm_val_R));

    delay(loop_del); // Delay to let PID code work its magic
  }
}
