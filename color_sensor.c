#include "color_sensor.h" // Include necessary header file

int i2c_fd; // File descriptor for I2C communication
uint8_t fd_open = 0; // Flag to track if file descriptor is open
uint8_t light = 0; // Variable to store light status

// Signal handler function to handle program exit
void exit_handler(int signum) {
    printf("\033[J\033[38;2;255;255;255mColor sensor off\n"); // Print message indicating color sensor is turned off
    printf("\033[?25h"); // Show cursor
    close(i2c_fd); // Close the file descriptor
}

// Function to read color data from the color sensor
void colors(ColorData *colorData) {
    t_raw_color raw_colors = {0}; // Structure to store raw color values
    t_proc_color proc_colors = {0}; // Structure to store processed color values
    unsigned char color[8]; // Array to store color data
    uint8_t error_measure = 0; // Error flag for measurement

    time_t start, end; // Variables to track time
    double elapsed_time; // Variable to store elapsed time

    i2c_fd = open(I2C_BUS, O_RDWR); // Open the I2C bus
    if (i2c_fd < 0) {
        perror("Failed to open I2C bus"); // Print error message if opening fails
        exit(1);
    } else {
        fd_open = 1; // Set flag to indicate file descriptor is open
    }

    // Set the I2C slave address
    if (ioctl(i2c_fd, I2C_SLAVE, COLOR_SENSOR_ADDR) < 0) {
        perror("Failed to set the I2C slave address"); // Print error message if setting slave address fails
        exit(1);
    }

    // Configure the color sensor
    write_register(i2c_fd, 0x80, 0x03); // Set sensor's operating mode to active data collection mode
    usleep(5000); // Wait for stabilization
    write_register(i2c_fd, 0x81, 0x00); // Disable interrupts for the sensor
    usleep(500);
    write_register(i2c_fd, 0x83, 0x07); // Set prescaler to 1 for maximum data acquisition rate
    usleep(500);
    write_register(i2c_fd, 0x8F, 0x00); // Disable interrupt signal locking for the sensor
    usleep(500);
    write_register(i2c_fd, 0x8D, 0x11); // Set integration time delay to 12 milliseconds
    usleep(500);
    write_register(i2c_fd, 0x80, 0x43); // Set sensor's operating mode to active data collection mode and enable light integration

    start = time(NULL); // Record start time
    while (color_sensor_alive) { // Main loop for continuous color sensing
        sleep(TIME_BETWEEN_MEASURES); // Wait for specified time between measurements

        if (!fd_open) { // Check if file descriptor is closed
            i2c_fd = open(I2C_BUS, O_RDWR); // Reopen the I2C bus if closed
            if (i2c_fd < 0) {
                perror("Failed to open I2C bus"); // Print error message if opening fails
                exit(1);
            }
        }

        end = time(NULL); // Record end time
        elapsed_time = difftime(end, start); // Calculate elapsed time

        if (elapsed_time >= 1.0) { // If one second has passed
            error_measure = 0; // Reset error flag
            atomic_exchange(&color_sensor_data_ready, 0); // Reset data ready flag

            // Read color data from sensor
            read_color(i2c_fd, color);

            // Extract raw color values from the read data
            raw_colors.clear = color[1] << 8 | color[0];
            raw_colors.red = color[3] << 8 | color[2];
            raw_colors.green = color[5] << 8 | color[4];
            raw_colors.blue = color[7] << 8 | color[6];

            // Calculate processed color values
            proc_colors.ir = (float)(raw_colors.red + raw_colors.green + raw_colors.blue - raw_colors.clear) / 2.0;
            proc_colors.clear = (float)(raw_colors.clear);
            proc_colors.red = (float)(raw_colors.red);
            proc_colors.green = (float)(raw_colors.green);
            proc_colors.blue = (float)(raw_colors.blue);

            // Convert processed color values to 8-bit representation if clear value is not zero
            if (proc_colors.clear != 0) {
                proc_colors.red = proc_colors.red * 255 / 65535.0;
                proc_colors.blue = proc_colors.blue * 255 / 65535.0;
                proc_colors.green = proc_colors.green * 255 / 65535.0;
            } else {
                error_measure |= 0x8; // Set error flag if clear value is zero
            }

            // Store processed color values in color data structure
            colorData->red = proc_colors.red;
            colorData->blue = proc_colors.blue;
            colorData->green = proc_colors.green;

            atomic_exchange(&color_sensor_data_ready, 1); // Set data ready flag
            start = end; // Reset start time
        }
    }

    //exit_handler(1); // Call exit handler function
}

// Function to write to color sensor registers
void write_register(int i2c_fd, unsigned char reg, unsigned char value) {
    struct i2c_rdwr_ioctl_data i2c_data; // Structure for I2C transaction
    struct i2c_msg msgs[1]; // Array of I2C messages

    unsigned char wr_buf[2]; // Buffer for write data

    wr_buf[0] = reg; // Register address
    wr_buf[1] = value; // Value to write

    msgs[0].addr = COLOR_SENSOR_ADDR; // Device address
    msgs[0].flags = 0; // Write operation
    msgs[0].len = 2; // Number of bytes to write
    msgs[0].buf = wr_buf; // Data buffer

    // Set up I2C transaction
    i2c_data.msgs = msgs;
    i2c_data.nmsgs = 1;

    // Send the I2C transaction
    ioctl(i2c_fd, I2C_RDWR, &i2c_data);
}

// Function to read color data from the color sensor
void read_color(int i2c_fd, unsigned char* buf) {
    struct i2c_msg messages[2]; // Array of I2C messages
    struct i2c_rdwr_ioctl_data ioctl_data; // Structure for I2C transaction

    unsigned char start_reg = 0x94; // Register address to start reading from

    messages[0].addr = COLOR_SENSOR_ADDR; // Device address
    messages[0].flags = 0; // Write operation
    messages[0].len = 1; // Number of bytes to write
    messages[0].buf = &start_reg; // Data buffer

    messages[1].addr = COLOR_SENSOR_ADDR; // Device address
    messages[1].flags = I2C_M_RD; // Read operation
    messages[1].len = 8; // Number of bytes to read (clear, red, green, blue)
    messages[1].buf = buf; // Data buffer to store read values

    ioctl_data.msgs = messages; // Set up I2C transaction
    ioctl_data.nmsgs = 2;

    // Perform the I2C transaction
    ioctl(i2c_fd, I2C_RDWR, &ioctl_data);
}


