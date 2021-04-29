// A simple control software for a robotic arm to run on a RPi.
// Reads messages from a serial port, and uses them to control servos with the PIGPIO library.

// To compile:
// g++ -Wall -pthread -o test main.cpp -lpigpio

// Note that the pigpio library is required to be installed, see http://abyz.me.uk/rpi/pigpio/



// C library headers
#include <stdio.h>  // for printf
#include <string.h> // for strerror and SIGINT

// Linux headers
#include <fcntl.h>   // for file control defines
#include <errno.h>   // for error functions
#include <termios.h> // for POSIX terminal control defines
#include <unistd.h>  // for POSIX OS API including IO

#include <sched.h>  // for scheduling
#include <pigpio.h> // for servo control
#include <signal.h> // for interrupt signal define

// C++ headers
#include <unordered_map> // for servo_map
using std::unordered_map;



// Define the known serial message offsets
#define ID_OFFSET 0
#define MOVEMENT_VALUE_OFFSET 2

// Define ServoIDs
enum ServoIDs
{
    BASE_DIR     = 0,
    LOWER_HEIGHT = 1,
    MID_HEIGHT   = 2,
    TOP_HEIGHT   = 3,
    CLAW_TILT    = 4,
    CLAW_GRAB    = 5
};


// Create a map of servos to their IDs
unordered_map<ServoIDs, uint8_t> servo_map = 
{
    {BASE_DIR    , 27},
    {LOWER_HEIGHT, 17},
    {MID_HEIGHT  , 22},
    {TOP_HEIGHT  , 23},
    {CLAW_TILT   , 24},
    {CLAW_GRAB   , 25}
};


// Global variable to stop the main update loop
int run = 1;



void stop(int signum)
{
   run = 0; // End the update loop by setting this var
}


int SetupSerial(int &serial_port_fd)
{
    struct termios tty; // Create new termios struct

    // Read in existing terminos settings
    // POSIX states that the struct passed to tcsetattr() must have been initialized with a call to tcgetattr()
    if(tcgetattr(serial_port_fd, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    // Setup the terminos struct
    tty.c_cflag &= ~PARENB;         // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;         // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;          // Clear all the size bits, then use one of the statements below
    tty.c_cflag |=  CS8;            // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;        // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |=  CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;  // Disable Canonical mode (input is processed when a new line character is received)
    tty.c_lflag &= ~ECHO;    // Disable echo
    tty.c_lflag &= ~ECHOE;   // Disable erasure
    tty.c_lflag &= ~ECHONL;  // Disable new-line echo
    tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON|IXOFF|IXANY);                              // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 0; // wait an infinite time until there is at least VMIN bytes avaliable
    tty.c_cc[VMIN]  = 1; // defines how many bytes to wait for

    cfsetspeed(&tty, B115200); // set both in and out baud rate

    return tcsetattr(serial_port_fd, TCSANOW, &tty); // Return the success of saving the terminos struct to the serial fd
}


int SetupAll(int &serial_port_fd)
{

    // Set thread to high priority
    const sched_param priority = {.sched_priority = 1}; // define a priority
    if (sched_setscheduler(0, SCHED_FIFO, &priority) != 0)  //if PID=0, sets policy and params for calling thread
    {
        printf("Error setting thread as high priority: %s\n", strerror(errno));
        return -1;
    }

    // Open serial port fd
    serial_port_fd = open("/dev/ttyACM0", O_RDWR);

    // Check for errors opening
    if (serial_port_fd < 0) 
    {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return -1;
    }

    // Setup the serial port parameters, check for errors saving the setup to the fd
    if(SetupSerial(serial_port_fd) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    // Setup the GPIO library
    if (gpioInitialise() < 0) 
        return -1;

    return gpioSetSignalFunc(SIGINT, stop); // registers a function to be called when a signal occurs (SIGINT = Ctrl + c)
}


void SetServosSafe()
{
    // Sleeps to avoid drawing too many amps
    gpioServo(servo_map.at(BASE_DIR), 1500);
    time_sleep(0.2);
    gpioServo(servo_map.at(LOWER_HEIGHT), 1500);
    time_sleep(0.2);
    gpioServo(servo_map.at(MID_HEIGHT), 1500);
    time_sleep(0.2);
    gpioServo(servo_map.at(TOP_HEIGHT), 1500);
    time_sleep(0.2);
    gpioServo(servo_map.at(CLAW_TILT), 1500);
    time_sleep(0.2);
    gpioServo(servo_map.at(CLAW_GRAB), 1500);
    time_sleep(0.2);
}



int main()
{
    // Will hold the fd of serial port
    int serial_port;

    // Perform setup of program
    if(SetupAll(serial_port) != 0)
        return 1;

    printf("Setting Servos to safe position.....\n");

    // set initial positions of servos to a known safe position
    SetServosSafe();
    
    printf("Entering update loop\n");

    char read_buf [20] = {0}; // buffer to hold serial reads

    while(run)
    {
        int num_bytes = read(serial_port, &read_buf, sizeof(read_buf)); // blocks until there is at least 1 byte to read (see terminos VMIN/VTIME)

        if(num_bytes > 0) // if there was a successful read
        {
            //printf("Read %i bytes. Received message: %s", num_bytes, read_buf);

            int id = read_buf[ID_OFFSET] - '0'; // obtain the servoID from the message
            char pulsewidth_str[5] = {0};

            for(int i = 0; i < 4; i++) // obtain the 4 character movement value from the message
            {
                pulsewidth_str[i] = read_buf[i + MOVEMENT_VALUE_OFFSET];
            }

            int pulsewidth = atoi(pulsewidth_str);

            printf("Servo: %d to PulseWidth: %d\n", (ServoIDs)id, pulsewidth);

            gpioServo(servo_map.at((ServoIDs)id), pulsewidth);
        }
        else if (num_bytes < 0) // if there was an error reading
        {
            printf("Error reading: %s", strerror(errno));
            run = 0;
        }
    }

    printf("\nEnding program\n");

    // Cleanup
    SetServosSafe();
    close(serial_port);
    gpioTerminate();

    return 0;
}


// From
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
// https://www.iot-programmer.com/index.php/books/22-raspberry-pi-and-the-iot-in-c/chapters-raspberry-pi-and-the-iot-in-c/33-raspberry-pi-iot-in-c-almost-realtime-linux?start=2