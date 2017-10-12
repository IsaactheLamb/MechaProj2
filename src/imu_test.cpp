//
// This code has been transcribed from 
// http://elinux.org/Interfacing_with_I2C_Devices
// for TRC3000, S2 2016
//

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <stdint.h>
#include <stdio.h>
#include <iostream>

using namespace std;

#define PI 3.1415926

int main(int argc, char **argv)
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

  int16_t acc[3] = {0, 0, 0};
  int16_t gyr[3] = {0, 0, 0};

  float acc_angle = 0;
  float gyr_rate = 0;

  float  CFangleX = 0;

  float G_GAIN = 0.07;
  float RAD_TO_DEG = 57.29578;
  float AA = 0.5;
  float DT = 0;

  long int last_time = 0;
  struct timespec gettime;

  float  CFangleY = 0; // Complementary filter angle

    for (;;)
    {

    	if (write(file,&start_addr,1) != 1) {
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

                printf("%6d %6d %6d %6d %6d %6d\n", gyr[0] , gyr[1] , gyr[2] , acc[0] , acc[1], acc[2]); 

	    gyr_rate += ((float) gyr[1] * G_GAIN );

	    acc_angle = (float) (atan2(acc[2], acc[0]) + M_PI)*RAD_TO_DEG;

	    last_time = gettime.tv_nsec;
	    clock_gettime(CLOCK_REALTIME, &gettime);    
	    if (gettime.tv_nsec > last_time)
	  {
	    	DT = (gettime.tv_nsec - last_time)/1000000000; // Convert nano to secs  
             cout << "DT " << (gettime.tv_nsec - last_time)/1000 << endl;
           }

	    CFangleY = AA*(CFangleY+gyr_rate*DT) +(1 - AA) * acc_angle;

    	cout << "AngleY: " <<  CFangleY << endl;

	}
	
	usleep(10000); // optional delay

  } // end of for

}

