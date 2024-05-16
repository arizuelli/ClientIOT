#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "clientsensor.h" // Include necessary header file

// Definitions for I2C communication and MPU6050 sensor address
#define I2C_BUS          "/dev/i2c-1"
#define MPU6050_ADDR     0x68

// Definitions for MPU6050 accelerometer register addresses
#define MPU6050_ACCEL_XOUT_H   0x3B
#define MPU6050_ACCEL_XOUT_L   0x3C
#define MPU6050_ACCEL_YOUT_H   0x3D
#define MPU6050_ACCEL_YOUT_L   0x3E
#define MPU6050_ACCEL_ZOUT_H   0x3F
#define MPU6050_ACCEL_ZOUT_L   0x40

#define TIME_BETWEEN_MEASURES 1 // Time between accelerometer measurements in seconds

extern float ax, ay, az; // External declaration of acceleration values
extern atomic_int acc_data_ready; // External declaration of data readiness flag
extern uint8_t accelerometer_alive; // External declaration of accelerometer status flag

// Function prototypes
void acceleration();
void write_acc_register(int file, unsigned char reg, unsigned char value);
void read_acceleration(int file, unsigned char* buf);
void stop_acc_measurements(); // Declaration of signal handler function

#endif /* ACCELEROMETER_H_ */

