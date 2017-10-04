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

#define PI 3.1415926

void main(void) {
    int file;
    char filename[40];
    unsigned char mode_change[2] = {0x00, 0x00}; // buffer for reg addr/value 
    unsigned char read_buffer[7] = {0x00};
    unsigned char start_addr = 0x28; 
 
    int addr = 0x6B;        // The I2C address of the LSM6DS33
    int idx;
    short Ax, Ay, Az;

    sprintf(filename,"/dev/i2c-2"); // using i2c-2 bus 
    if ((file = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }
    else 
	printf("%s is now open successfully\n", filename);

    if (ioctl(file,I2C_SLAVE,addr) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }
    else 
	printf("I2C bus access to LSM6DS33 is acquired\n");
	
	mode_change[0] = 0x10; // CTRL1_XL address
	mode_change[1] = 0x80; // Reg value for 1.66KHz sampling 

    if (write(file,mode_change,2) != 2) {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to mode change.\n");
        exit(1);
    }
    else
	printf("Accelerometer enabled at 1.66KHz mode.\n");
 
    usleep(100000); // waiting for the sensor to boot up

    for (idx = 0;idx < 1000; idx++)
    {

    	if (write(file,&start_addr,1) != 1) {
        	/* ERROR HANDLING: i2c transaction failed */
        	printf("Failed to change address pointer.\n");
        	exit(1);
    	}

	if (read(file, read_buffer, 6) !=6 )
	{
		printf("Something's wrong, received data length is not 6\n");
	}
	else 
	{
		// Assemble data into short integer
		Ax = (short)(read_buffer[1] << 8 | read_buffer[0]);
		Ay = (short)(read_buffer[3] << 8 | read_buffer[2]);
		Az = (short)(read_buffer[5] << 8 | read_buffer[4]);
                printf("%6d %6d %8d\n", Ax, Ay, Az); 

	}
	
	usleep(10000); // optional delay

  } // end of for

}

