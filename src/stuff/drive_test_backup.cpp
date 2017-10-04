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

static volatile int X = 0; 
static volatile int X_prev = 0; 

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
    cout << "\tRPM Left  :" << (int) rpm_L << endl;
    cout << "\t\tRPM Right :" << (int) rpm_R << endl;
  }
}

PI_THREAD (imuThread)
{  
  while(1)
  {
	  scanf("%f *", &angle);
	  angle = angle + 0.08;

	  cout << "\tAngle is " << angle << endl;

	  // Read to end of line so don't get stuck on one invalid line.
	  while(getc(stdin) != '\n');

    //delay(loop_del);
  }
}

PI_THREAD (VisionThread)
{  
	VideoCapture cap(0); //capture the video from web cam
	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		
	}
	
	vector <Vec3f> v3fCircles;	//3 element vector of floats, this will be the pass by reference output of HoughCircles()
	
	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	
	int iLowH = 0;		//0
	int iHighH = 40;	//179
	int iLowS = 121;	//0
	int iHighS = 255;	//255
	int iLowV = 143;	//0
	int iHighV = 255;	//255

	while (true)
	{
		Mat imgOriginal;
		
		bool bSuccess = cap.read(imgOriginal); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
		cout << "Cannot read a frame from video stream" << endl;
		break;
		}
		

		Mat imgHSV;
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

		
		Mat imgThresholded;
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

		//morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		
		GaussianBlur(imgThresholded, imgThresholded, Size(5, 5), 0);	//Blur Effect																						  fill circles vector with all circles in processed image
		
		HoughCircles(imgThresholded, v3fCircles, CV_HOUGH_GRADIENT, 2, imgThresholded.rows / 4, 100, 50, 10, 800);	// alogarithm for detecting circles
		
    		 X_prev = X;		

		for (int i = 0; i < v3fCircles.size(); i++)		// for each circle
		{      
			X =v3fCircles[i][0] ; 

			delay(loop_del);
		}		

		
	}
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
  //piThreadCreate (VisionThread); // Start Vision thread
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

  // Direction of RPM
  int dirs[2] = {-1, 1};

  for(;;) 
  {
    // *********************** IMU CONTROL ***********************
    des_rpm = (angle < 0)*(-1)*sqrt(abs(angle))*MTR_MAX_RPM + (angle > 0)*sqrt(abs(angle))*MTR_MAX_RPM;
    cout << "\tdesired RPM is " << des_rpm << endl;

    // ******************** DRIVE WHEEL PID CONTROL ********************
    bool force_dir_L = 0;
    bool force_dir_R = 0;

    //cout <<"\t\t\tBall position: X = " << X << endl; // x position of center point of
    //cout <<"\t\tBall position: X_prev = " << X_prev << endl; // x position of center point of circle

    if (X != X_prev)
    {
	if (X>=320)	// target on the right
	{
		force_dir_L =  1;
	    	force_dir_R =  0;		}

	else	 // target on the left
	{	force_dir_L =  0;
	    	force_dir_R =  1;
	 }
    }
   else 
   {
	//cout << " RUNNING ELSE CASE &%&%&%&%&%&%&%&%" << endl;
	force_dir_L = 1;
	force_dir_R = 1;
    }

    double out_L = pid_L.getOutput(rpm_L, dirs[force_dir_L]*des_rpm); // Get delta in PWM
    double out_R = pid_R.getOutput(rpm_R, dirs[force_dir_R]*des_rpm); // Get delta in PWM

    pwm_val_L += out_2_in*out_L; // Increment PWM
    pwm_val_R += out_2_in*out_R; 

    

    // Set directions
    dir_L = pwm_val_L >= 0;
    dir_R = pwm_val_R >= 0;

    //cout << "\tDir Left is " << dir_L  << "\t\tDir right is " << dir_R << endl;		
	
    // Constrain PWM values
    if (pwm_val_L > MAX_PWM) pwm_val_L = MAX_PWM;
    else if (pwm_val_L < -MAX_PWM) pwm_val_L = -MAX_PWM;

    if (pwm_val_R > MAX_PWM) pwm_val_R = MAX_PWM;
    else if (pwm_val_R < -MAX_PWM) pwm_val_R = -MAX_PWM;

    cout << "WRITING TO PWMS *******************************************************" << endl;
    if (abs(angle)>0.5)
	{
		pwm_val_L=0;
		pwm_val_R=0;
	}
    // Set PWMs
    softPwmWrite(PWM_PIN_L1, (int) dir_L*fabs(pwm_val_L)); 
    softPwmWrite(PWM_PIN_L2, (int) (!dir_L)*fabs(pwm_val_L));
    softPwmWrite(PWM_PIN_R1, (int) dir_R*fabs(pwm_val_R)); 
    softPwmWrite(PWM_PIN_R2, (int) (!dir_R)*fabs(pwm_val_R));

    delay(loop_del); // Delay to let PID code work its magic
  }
}
