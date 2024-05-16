#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include "clientsensor.h" // Include necessary header file

// Definitions for I2C communication and color sensor address
#define I2C_BUS             "/dev/i2c-1"
#define COLOR_SENSOR_ADDR   0x29

// Time between color measurements in seconds
#define TIME_BETWEEN_MEASURES 1

// Function prototypes
void write_register(int file, unsigned char reg, unsigned char value);
void read_color(int file, unsigned char* buf);
void init_signals();
void restore_signals();
void toggle_flash(uint8_t *flash);

// External declarations for flags and messages
extern uint8_t color_sensor_alive;
extern atomic_int color_sensor_data_ready;
extern char color_sensor_msg[1500];

// Structure to store raw color values
typedef struct {
    uint16_t clear;     // Clear
    uint16_t red, green, blue; // RGB values
} t_raw_color;

// Structure to store processed color values
typedef struct {
    float ir;
    float clear;
    float red, green, blue; // RGB values
} t_proc_color;

// Structure for RGB data
typedef struct {
    float red;
    float green;
    float blue;
} ColorData;

// Function to read color data from the sensor
void colors(ColorData *colorData);

#endif /* COLOR_SENSOR_H_ */

