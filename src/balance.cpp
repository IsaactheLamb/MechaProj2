#include <math.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <fstream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

extern "C" 
{
	#include "RPIO/pwm.h"
}

 #include <signal.h> 
#include <sys/types.h>

#include <wiringPi.h>
#include <softPwm.h>

#include "MiniPID/MiniPID.h"
#include <time.h>

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

#define ENC_HZ 100

#define MAX_PWM 100
#define MTR_MAX_RPM 110
#define PI 3.1415926

static volatile int enc_cnt_L = 0;
static volatile int enc_cnt_R = 0;

static volatile float rpm_L = 0;
static volatile float rpm_R = 0;

static volatile bool dir_L = 0; // Wheel directions
static volatile bool dir_R = 0;

// Desired RPM values for each wheel
static volatile float des_rpm_L = 0.0; 
static volatile float des_rpm_R = 0.0; 

// Angle tracking
static volatile float angle = 0;
static volatile float  CFangleY = 0; // Complementary filter angle

const float loop_del = 1000.0/ENC_HZ;

static volatile int X = 0; 
static volatile int X_prev = 0; 

ofstream myfile; // csv recording file

// Function for clamping a value between an upper and lower bound
double clamp(double val, double upper, double lower) 
{
  return min(upper, max(val, lower));
}

// Function to return sign of a number
int sign(double val) 
{
  return (val > 0) - (val < 0);
}

// Encoder interrupt service routine for left wheel
void encISR_L() 
{
  ++enc_cnt_L;
}

// Encoder interrupt service routine for right wheel
void encISR_R() 
{
  ++enc_cnt_R;
}

PI_THREAD (encThread)
{
  // Set up encoder pins for interrupts
  wiringPiISR(ENC_PIN_L1, INT_EDGE_BOTH, &encISR_L);
  wiringPiISR(ENC_PIN_L2, INT_EDGE_BOTH, &encISR_L);
  wiringPiISR(ENC_PIN_R1, INT_EDGE_BOTH, &encISR_R);
  wiringPiISR(ENC_PIN_R2, INT_EDGE_BOTH, &encISR_R);

  float del_time = 1.0/((float)ENC_HZ);
  float ppr = 150*12; // Pulses per revolution at output shaft

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
    
    cout << "\t\t\t RPM Left  :" << rpm_L << endl;
    cout << "\t\t\t RPM Right :" << rpm_R << endl;
   //cout << "\t\t\t ENC Left  :" << enc_cnt_L << endl;
   //cout << "\t\t\t ENC Right :" << enc_cnt_R << endl;
  }
}

PI_THREAD(cvThread)
{
	VideoCapture cap(0); //capture the video from web cam
	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
	}
	
	int iLowH = 0;	//0
	int iHighH = 24;	//179

	int iLowS = 68;	//0
	int iHighS = 186;	//255

	int iLowV = 162;	//0
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

		GaussianBlur(imgThresholded, imgThresholded, Size(3, 3), 0);	//Blur Effect

		vector <vector<Point> > contours;
		vector <Vec4i> hierarchy;

		Canny(imgThresholded, imgThresholded, 50, 120, 3);
		findContours(imgThresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		// Get the moments of image
		vector <Moments> mu(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			mu[i] = moments(contours[i], false);
		}

		X_prev = X;

		// Get the mass centres
		vector <Point2f> mc(contours.size());
		for (int i = 0; i < contours.size(); i++)
		{
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
			X = mc[i].x;
			delay(loop_del);
		}
	}
}

PI_THREAD (imuThread)
{  
    int file;
    char filename[40];
    unsigned char mode_change[2] = {0x00, 0x00}; // buffer for reg addr/value 
    uint8_t read_buffer[13] = {0x0000};
    unsigned char start_addr = 0x22; 
 
    int addr = 0x6B;        // The I2C address of the LSM6DS33
    int idx;
    int16_t Gx, Gy, Gz;
    int16_t Ax, Ay, Az;

    sprintf(filename,"/dev/i2c-1"); // using i2c-2 bus 
    if ((file = open(filename, O_RDWR)) < 0) {
        printf("Failed to open the bus.");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }
    else 
	printf("%s is now open successfully\n", filename);

    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }
    else 
	printf("I2C bus access to LSM6DS33 is acquired\n");
	
    mode_change[0] = 0x10; // CTRL1_XL address
    mode_change[1] = 0x8C; // Reg value for 1.66KHz sampling 

    if (write(file,mode_change,2) != 2) {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to mode change.\n");
        exit(1);
    }
    else
	printf("Accelerometer enabled at 1.66KHz mode.\n");

	usleep(100000); // waiting for the sensor to boot up


    mode_change[0] = 0x11; // CTRL2_G address
    mode_change[1] = 0x8C; // Reg value for 1.66KHz sampling 

    if (write(file,mode_change,2) != 2) {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to mode change.\n");
        exit(1);
    }
    else
	printf("Gyro enabled at 1.66KHz mode.\n");


 
    usleep(100000); // waiting for the sensor to boot up
  float acc[3] = {0, 0, 0};
  float gyr[3] = {0, 0, 0};

  float acc_angle = 0;
  float gyr_rate = 0;

  float  CFangleX = 0;

  float G_GAIN = 0.07;
  float RAD_TO_DEG = 57.29578;
  float AA = 0.5;
  float DT = 0;

  long int last_time = 0;
  struct timespec gettime;

  for(;;)
  {

    	if (write(file,&start_addr,1) != 1) 
	{
        	/* ERROR HANDLING: i2c transaction failed */
        	printf("Failed to change address pointer.\n");
        	exit(1);
    	}

	if (read(file, read_buffer, 12) !=12 )
	{
		printf("Something's wrong, received data length is not 12\n");
	}
	else 
	{
		// Assemble data into short integer
		gyr[0] = (int16_t) (read_buffer[1] << 8 | read_buffer[0]);
		gyr[1]= (int16_t) (read_buffer[3] << 8 | read_buffer[2]);
		gyr[2] = (int16_t) (read_buffer[5] << 8 | read_buffer[4]);

		acc[0] = (int16_t) (read_buffer[7] << 8 | read_buffer[6]);
		acc[1] = (int16_t) (read_buffer[9] << 8 | read_buffer[8]);
		acc[2] = (int16_t) (read_buffer[11] << 8 | read_buffer[10]);

             //   printf("%6d %6d %6d %6d %6d %6d\n", gyr[0] , gyr[1] , gyr[2] , acc[0] , acc[1], acc[2]); 

    		gyr_rate += ((float) gyr[1] * G_GAIN );

    		acc_angle = (float) (atan2(acc[2], acc[0]) + M_PI)*RAD_TO_DEG;

    		last_time = gettime.tv_nsec;
    		clock_gettime(CLOCK_REALTIME, &gettime);    
    		if (gettime.tv_nsec > last_time)
		{
    		DT = (gettime.tv_nsec - last_time)/1000000000; // Convert nano to secs
		cout << "Angle DT: " <<  (gettime.tv_nsec - last_time)/1000 << " ms" << endl;
		}

    		CFangleY = AA*(CFangleY+gyr_rate*DT) +(1 - AA) * acc_angle;

    		//cout << "\t\t\t\t\t AngleY: " <<  CFangleY << endl;
    		//while(getc(stdin) != '\n');   // Read to end of line to not get stuck
  	}
	usleep(2500); // optional delay
   }
}
struct sigaction old_action;

void sigint_handler(int sig_no)
{
//	struct sigaction old_action;
    cout << "CTRL-C pressed" << endl; 
    sigaction(SIGINT, &old_action, NULL);

    myfile.close();

    kill(0, SIGINT);
}

int main(int argc, char **argv)
{
  wiringPiSetup();              // Initialise wiringPi
  piThreadCreate (imuThread);   // Start IMU angle thread
  piThreadCreate(cvThread);      // Start openCV thread
  piThreadCreate (encThread); // Start encoder thread

  //int pwm_period = 20000;
  //setup(PULSE_WIDTH_INCREMENT_GRANULARITY_US_DEFAULT, DELAY_VIA_PWM);
  //init_channel(8, pwm_period );
  //init_channel(9, pwm_period );
  //init_channel(10, pwm_period );
  //init_channel(11, pwm_period );

  // Setup for graceful exit, closing CSV file before exit
	struct sigaction action;
	memset(&action, 0, sizeof(action));
	action.sa_handler = &sigint_handler;
	sigaction(SIGINT, &action, &old_action);

  // Set default PID gains
  double s_p = 0.4;
  double s_i = 0;
  double s_d = 0;

   double b_p = 0;
   double b_i = 0;
   double b_d = 0;

  // Grab desired PID gains from command line
  if (argc == 7) {
    b_p = stod(argv[1]);
    b_i = stod(argv[2]);
    b_d = stod(argv[3]);

    s_p = stod(argv[4]);
    s_i = stod(argv[5]);
    s_d = stod(argv[6]);
  }

  MiniPID pid_L = MiniPID(s_p, s_i, s_d);
  MiniPID pid_R = MiniPID(s_p, s_i, s_d);
  pid_L.setOutputLimits(MAX_PWM); 
  pid_R.setOutputLimits(MAX_PWM); 
  
  MiniPID pid = MiniPID(b_p, b_i, b_d); // Initialise the PID controller
  pid.setOutputLimits(MAX_PWM);       // Limit PID output to part of PWM range

  double output = 0;   // PID output variable
  double setpoint = 0; // Zero degrees, PID setpoint value

  double deadband = 1;    // PWM values for which we want to clip to zero 
  double sp_incr = 0.001; // Value to increment setpoint by

  float turn_incr = 1;
  float forward_spd = 6;

  float pwm_val_L = 0;  // PWM value for left wheel (0 to 100)
  float pwm_val_R = 0; // PWM value for right wheel (0 to 100)

  // Initialise pins for PWM control of wheels
  softPwmCreate (PWM_PIN_L1, 0, MAX_PWM);
  softPwmCreate (PWM_PIN_L2, 0, MAX_PWM);
  softPwmCreate (PWM_PIN_R1, 0, MAX_PWM);
  softPwmCreate (PWM_PIN_R2, 0, MAX_PWM);

  myfile.open("bal_data.csv");
  myfile << s_p << " " << s_i << " " << s_d << "\n"; // Print PID values to first row
  // Columns: d_t, angle, rpm_l, rpm_r, pid_output

   long int last_time = 0;
   struct timespec gettime;
   float DT = 0;

   float calib = 94.7;

  for(;;) // ZE MAIN LOOP
  {
    angle = CFangleY - calib;
   cout << "\t\t\t\t\t Angle: " <<  angle << endl;

    // **** PID CONTROL *************************************************
    output = pid.getOutput(CFangleY - calib, setpoint); // Perform PID calculation
    cout << "PID output: " << output << endl;

    if (abs(output) < deadband) 
    {
	//output = 0;  // Clip output to deadband
    }
    else
    {
	//output = output - sign(output)*deadband;
    }
    
    // **** DIRECTION AND BOUNDARIES ************************************
    if (X != X_prev) 
    {
	if (X <= 256)	// target on the right
	{
	     des_rpm_R = 28;
             des_rpm_L = 20;
             //cout << "left" << endl;
	}
	else if (X >= 384) // target on the left
	{
	    des_rpm_R = 20;
            des_rpm_L = 28;
            //cout << "right" << endl;
	}
        else
	{
	   des_rpm_R = 24;
           des_rpm_L = 24;
            // cout << "straight" << endl;
	}
    }

        pwm_val_L = pid_L.getOutput(rpm_L, output); //*(MAX_PWM/pid_L.getOutput(0, 100));
        pwm_val_R = pid_R.getOutput(rpm_R, output);//*(MAX_PWM/pid_R.getOutput(0, 100));

	cout << "\t PWM LEFT: " << pwm_val_L << endl;
       cout << "\t PWM RIGHT: " << pwm_val_R << endl;

   // cout << "\t X = " << X << endl;

    dir_L = pwm_val_L >= 0; // Set directions
    dir_R = pwm_val_R >= 0;	

    // If angle is greater than ... degrees, stop motors
    if (abs(angle) > 50) 
	  {
		  pwm_val_L = 0;
		  pwm_val_R = 0;
	  }

    // If we are moving in one direction, adjust setpoint in same direction
    // to compensate.
    //setpoint = setpoint - sign(output)*sp_incr; 

    // **** DRIVE THE MOTORS ********************************************
    softPwmWrite(PWM_PIN_L1, (int) dir_L*fabs(pwm_val_L));    // Set PWMs
    softPwmWrite(PWM_PIN_L2, (int) (!dir_L)*fabs(pwm_val_L));
    softPwmWrite(PWM_PIN_R1, (int) dir_R*fabs(pwm_val_R)); 
    softPwmWrite(PWM_PIN_R2, (int) (!dir_R)*fabs(pwm_val_R));

    //add_channel_pulse(8, PWM_PIN_L1, 0, (int) dir_L*fabs(pwm_val_L));
    //add_channel_pulse(9, PWM_PIN_L2, 0, (int) (!dir_L)*fabs(pwm_val_L));
    //add_channel_pulse(10, PWM_PIN_R1, 0, (int) dir_R*fabs(pwm_val_R));
    //add_channel_pulse(11, PWM_PIN_R2, 0, (int) (!dir_R)*fabs(pwm_val_R));

    last_time = gettime.tv_nsec;
    clock_gettime(CLOCK_REALTIME, &gettime);    
    if (gettime.tv_nsec > last_time)
  	DT = (gettime.tv_nsec - last_time)/1000000; // Convert nano to millisecs

     myfile << DT << ", " << CFangleY - calib << ", " << rpm_L << ", " << rpm_R << ", " << output << "\n";

    delay(loop_del); // Delay to let PID code work its magic
  }
}
