#include "accelerometer.h" // Include necessary header file

int fd; // Global variable for file descriptor

// Signal handler function to stop accelerometer measurements
void stop_acc_measurements(int signum) {
    printf("\033[KAccelerometer off\n"); // Print message indicating accelerometer is off
    close(fd); // Close the file descriptor
}

// Function to read acceleration data
void acceleration() {
    unsigned char rd_buf[6]; // Buffer to store data read from sensor

    // Open the I2C bus
    fd = open(I2C_BUS, O_RDWR);
    if (fd < 0) {
        perror("Failed to open I2C bus"); // Print error message if opening fails
        exit(1);
    }

    // Set the I2C slave address
    if (ioctl(fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
        perror("Failed to set the I2C slave address"); // Print error message if setting slave address fails
        exit(1);
    }

    // Configure MPU6050 sensor
    write_acc_register(fd, 0x6b, 0x80); // Perform initial reset
    usleep(5000); // Wait for stabilization
    // Configure accelerometer settings
    write_acc_register(fd, 0x6c, 0xC7);
    usleep(500);
    // Wake up the sensor from sleep mode
    write_acc_register(fd, 0x6b, 0x28);
    usleep(500);
    // Configure digital low-pass filter settings
    write_acc_register(fd, 0x1a, 0x03);
    usleep(500);
    // Set sample rate divider
    write_acc_register(fd, 0x19, 0x08);

    // Main loop for continuous reading of acceleration data
    while (accelerometer_alive) {
        sleep(TIME_BETWEEN_MEASURES); // Wait for specified time between measurements

        // Read acceleration data from sensor
        read_acceleration(fd, rd_buf);

        // Process the data
        atomic_exchange(&acc_data_ready, 0); // Reset data ready flag
        // Extract acceleration values from the read buffer
        ax = (rd_buf[0] << 8) | rd_buf[1];
        ay = (rd_buf[2] << 8) | rd_buf[3];
        az = (rd_buf[4] << 8) | rd_buf[5];
        // Convert raw values to acceleration in g
        ax = (signed short)ax / 16384.0;
        ay = (signed short)ay / 16384.0;
        az = (signed short)az / 16384.0;

        // Send data to main
        atomic_exchange(&acc_data_ready, 1); // Set data ready flag
    }
    close(fd); // Close the file descriptor
}

// Function to write to accelerometer registers
void write_acc_register(int file, unsigned char reg, unsigned char value) {
    // Define I2C message and data structures
    struct i2c_rdwr_ioctl_data i2c_data;
    struct i2c_msg msgs[1];
    unsigned char wr_buf[2];

    // Prepare data to be written
    wr_buf[0] = reg; // Register address
    wr_buf[1] = value; // Value to write

    // Set up I2C message
    msgs[0].addr = MPU6050_ADDR; // Device address
    msgs[0].flags = 0; // Write operation
    msgs[0].len = 2; // Number of bytes to write
    msgs[0].buf = wr_buf; // Data buffer

    // Set up I2C transaction
    i2c_data.msgs = msgs;
    i2c_data.nmsgs = 1;

    // Send the I2C transaction
    ioctl(file, I2C_RDWR, &i2c_data);
}

// Function to read acceleration from the sensor
void read_acceleration(int file, unsigned char* buf) {
    // Define I2C message and data structures
    struct i2c_msg messages[2];
    struct i2c_rdwr_ioctl_data ioctl_data;
    unsigned char start_reg = 0x3b;

    // Write operation: set register to start reading from
    messages[0].addr = 0x68; // MPU-6050 device address
    messages[0].flags = 0; // Write operation
    messages[0].len = 1; // Number of bytes to write
    messages[0].buf = &start_reg; // Data buffer

    // Read operation: read acceleration data
    messages[1].addr = 0x68; // MPU-6050 device address
    messages[1].flags = I2C_M_RD; // Read operation
    messages[1].len = 6; // Number of bytes to read
    messages[1].buf = buf; // Data buffer to store read values

    // Set up I2C transaction
    ioctl_data.msgs = messages;
    ioctl_data.nmsgs = 2;

    // Perform the I2C transaction
    ioctl(file, I2C_RDWR, &ioctl_data);
}


