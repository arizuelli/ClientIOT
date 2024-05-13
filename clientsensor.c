#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "clientsensor.h"
#define MAIN_H
#include "color_sensor.h"
#include "accelerometer.h"

#define WRITE_BUF_SIZE     1500
#define BUF_SIZE           256
#define SERVER_IP          "192.168.1.41"
#define PORT               8888

#define BUFFER_SIZE        1024
#define DATA_COUNT         10             // Total count of measures between sends to server
#define TIME_BETWEEN_MEASURES 1           // Time between each measure

// Function prototypes for starting sensor processes
void start_accelerometer();
void start_color_sensor(ColorData *colorData);

// Threads for sensors
pthread_t thread_color_sensor, thread_acc;
int thread_error;
uint8_t color_sensor_alive, accelerometer_alive;

// Atomic variables to indicate whether sensor data is ready
atomic_int color_sensor_data_ready = 0;
atomic_int acc_data_ready = 0;

// Structure for storing RGB values
ColorData colorData = {0};

// Acceleration values
float ax, ay, az;

// Function prototypes for representing data
void init_signals();
void restore_signals();
struct termios original_termios = {0}, modified_termios = {0};

// Signal handler for handling exit signal (CTRL + C)
void sigint_handler(int signum) {
    int time = 1;           // Original time required to close
    printf("Exiting, wait until everything is closed...\n");
    if(accelerometer_alive) {
        accelerometer_alive = 0;
        stop_acc_measurements(1); // Stop accelerometer measurements
        time += 3;
    }
    if(color_sensor_alive) {
        color_sensor_alive = 0;
        exit_handler(1); // Handle exit for color sensor
        time += 3;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios); // Restore original terminal settings
    sleep(time);
    exit(1);
}

// Collect sensor data with a 1-second interval for 10 seconds and write it to the SensorData array
void collectSensorData(SensorData* data) {
    for (int i = 0; i < DATA_COUNT; i++) {
        data[i].red = colorData.red;
        data[i].green = colorData.green;
        data[i].blue = colorData.blue;
        data[i].acceleration_x = ax;
        data[i].acceleration_y = ay;
        data[i].acceleration_z = az;
        sleep(TIME_BETWEEN_MEASURES);
    }
}

// Print collected Sensor data values
void printSensorDataArray(const SensorData* data, int count) {
    for (int i = 0; i < count; i++) {
        printf("Data %d:\n", i + 1);
        printf("Acceleration X: %.2f\n", data[i].acceleration_x);
        printf("Acceleration Y: %.2f\n", data[i].acceleration_y);
        printf("Acceleration Z: %.2f\n", data[i].acceleration_z);
        printf("Red: %.2f\n", data[i].red);
        printf("Green: %.2f\n", data[i].green);
        printf("Blue: %.2f\n", data[i].blue);
        printf("\n");
    }
}

// Receive and print server response
void receiveServerResponse(int client_fd){
    char buffer[BUF_SIZE];
    int read_size;
    read_size = recvfrom(client_fd, buffer, BUF_SIZE, 0, NULL, NULL);
    if (read_size == -1) {
        perror("recvfrom failed");
        exit(EXIT_FAILURE);
    }
    printf("Received response from server - %s\n", buffer);
}

int main() {
    int client_fd;
    struct sockaddr_in server_addr;

    // Initialize signals
    init_signals();

    // Array for storing sensor data
    SensorData sensor_data[DATA_COUNT];

    // Buffer for storing sensor data to be sent to the server
    char sensor_data_buffer[BUFFER_SIZE];

    // Create socket
    if ((client_fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Prepare the server address structure
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    server_addr.sin_port = htons(PORT);
    printf("Send\n");
    signal(SIGINT, sigint_handler); // Register signal handler for exit signal (CTRL + C)

    // Send "hello" message to server
    if (sendto(client_fd, "hello from client", strlen("hello from client"), 0, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
        perror("sendto failed");
        exit(EXIT_FAILURE);
    }

    // Receive response message from server
    receiveServerResponse(client_fd);

    // Main loop
    while(1) {
        usleep(10000);

        // Start sensor processes in threads
        thread_error = pthread_create(&thread_color_sensor, NULL, &start_accelerometer, NULL);
        accelerometer_alive = 1;
        thread_error = pthread_create(&thread_acc, NULL, &start_color_sensor, &colorData);
        color_sensor_alive = 1;

        // Represent data
        if(atomic_load(&acc_data_ready) && atomic_load(&color_sensor_data_ready)) {
            collectSensorData(&sensor_data);
            printSensorDataArray(&sensor_data, DATA_COUNT);
            memcpy(sensor_data_buffer, sensor_data, sizeof(sensor_data)); // Serialize data

            // Send sensor data to the server
            if (sendto(client_fd, sensor_data_buffer, sizeof(sensor_data), 0, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1) {
                perror("sendto failed");
                exit(EXIT_FAILURE);
            }
            receiveServerResponse(client_fd);

            fflush(stdout);
        }
    }

    // Close the socket
    close(client_fd);

    return 0;
}

// Start color sensor
void start_color_sensor(ColorData* colorData) {
    colors(colorData);
}

// Start accelerometer
void start_accelerometer() {
    acceleration();
}

// Initialize signals
void init_signals() {
    // Save the original terminal settings
    tcgetattr(STDIN_FILENO, &original_termios);

    // Copy the original settings and modify them
    modified_termios = original_termios;
    modified_termios.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
    modified_termios.c_cc[VMIN] = 0;    // Set VMIN to 0 to read input without blocking
    modified_termios.c_cc[VTIME] = 0;   // Disable timeout

    // Set the modified terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &modified_termios);

    // Set stdin to non-blocking mode
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    flags |= O_NONBLOCK;
    fcntl(STDIN_FILENO, F_SETFL, flags);
}

// Restore original signals
void restore_signals() {
    // Set the modified terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
}
