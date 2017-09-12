#include <math.h>
#include <stdio.h>
#include <iostream>

#include <wiringPi.h>
#include <softPwm.h>

#include "MiniPID/MiniPID.h"

using namespace std;

#define PWM_PIN_1 15
#define ENC_PIN 16
#define ENC_PIN_2 1

#define ENC_HZ 10

#define MAX_PWM 1000
#define MTR_MAX_RPM 85

static volatile int enc_count = 0;
static volatile float rpm = 0;

void encISR() 
{
  ++enc_count;
}

PI_THREAD (encThread)
{
  wiringPiISR(ENC_PIN, INT_EDGE_RISING, &encISR);
  wiringPiISR(ENC_PIN_2, INT_EDGE_RISING, &encISR);

  float del_time = 1.0/((float)ENC_HZ);
  float ppr = 150*6; // Pulses per revolution at output shaft
  
  for (;;)
  {
    //cout << "Encoder count: " << enc_count << endl;
    enc_count = 0;
    delay(1000.0*del_time); // Delay in ms between readings

    rpm = 60.0*((enc_count/ppr)/del_time);

    //printf("Angular velocity: %.2f rad/s.", ang_vel);
    cout << "RPM :" << (int) rpm << endl;
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

  MiniPID pid = MiniPID(c_p, c_i, c_d); // TODO tune

  float des_rpm = 20.0; // Desired RPM value
  float pwm_val = 0; // PWM value to write to pin

  // pin: GPIO 14, initial val: 0, pwm range: 100 (treating as %)  
  softPwmCreate (PWM_PIN_1, 0, MAX_PWM) ;

  for(;;) 
  {
    double output = pid.getOutput(rpm, des_rpm); // Get amount to change PWM by
    //cout << "PID Output: " << output << endl; 	

    pwm_val += (output/MTR_MAX_RPM)*MAX_PWM; // Increment PWM

    if (pwm_val > MAX_PWM) pwm_val = MAX_PWM;
    else if (pwm_val < 0) pwm_val = 0;

    softPwmWrite(PWM_PIN_1, (int) pwm_val); // Set PWM

    //cout << "PWM val: " << pwm_val << endl;

    delay(1000.0/ENC_HZ); // Delay to let PID code work its magic

  }
}
