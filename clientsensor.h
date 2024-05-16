#ifndef MAIN_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <termios.h>
#include <pthread.h>
#include <stdatomic.h>

// Define a structure to hold sensor data to be sent to the server
typedef struct {
    float acceleration_x;   // Acceleration along the X-axis
    float acceleration_y;   // Acceleration along the Y-axis
    float acceleration_z;   // Acceleration along the Z-axis
    float red;              // Red component of color
    float green;            // Green component of color
    float blue;             // Blue component of color
} SensorData;

#endif /* MAIN_H_ */
