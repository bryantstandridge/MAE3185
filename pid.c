#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/timer.h>
#include <hardware/i2c.h>

// TOUCHSCREEN
#define SCL 1 // Define GPIO for i2c0 SCL Line
#define SDA 0 // Define GPIO for i2c0 SDA Line
#define addr 0x48 // touchscreen target address both A0 and A1 are held low making address 1001000
const int Lx = 165;
const int Ly = 105;

// SERVO MOTOR
#define servoOnePin 14                                          // gpio pin for servo number one
#define servoTwoPin 15                                          // gpio pin for servo number two

// VARIABLES
uint8_t snd[1] = {0xB0}; // 10110000 pg.25
uint8_t x_pos[1] = {0xC0};
uint8_t y_pos[1] = {0xD0};
uint8_t x[2] = {0, 0};
uint8_t y[2] = {0, 0};

float pos_x = 0;  // creates a variable that reads x position
float pos_y = 0;  // creates a variable that reads y position

uint xcc = 0;       // variable for the x axis cc value
uint ycc = 0;       // variable for the y axis cc value

const uint touchdt = 5000;      // variable for how often the read touchscreen task should be executed
const uint motordt = 20000;     // variable for how often the control motors task should be executed
uint lasttouchcalled = 0;       // variable for the last time the read touchscreen task was executed
uint lastmotorcalled = 0;       // variable for the last time the control motors task was executed

float dt = 0.0;              // variable for the differential amount of time
float dxerror = 0.0;         // variable for the derivative of the x axis error
float dyerror = 0.0;         // variable for the derivative of the y axis error
float ixerror = 0.0;         // variable for the integral of the x axis error
float iyerror = 0.0;         // variable for the integral of the y axis error

float xcurrent = 0.0;    // variable for the current x axis position of the ball                                          
float ycurrent = 0.0;    // variable for the current y axis position of the ball
float xdesired = 0.0;    // variable for the desired x axis position of the ball
float ydesired = 0.0;    // variable for the desired y axis position of the ball

float currentxerror = 0.0;       // variable for the current x axis error (distance between the desired position and the actual position)
float currentyerror = 0.0;       // variable for the curent y axis error (distance between the desired position and the actual position)
float lastxerror = 0.0;      // variable for the last x axis error (distance between the desired position and the actual position)
float lastyerror = 0.0;      // variable for the last y axis error (distance between the desired position and the actual position)

float taux = 0.0;           // variable for tau x
float tauy = 0.0;           // variable for tau y

float kp = 0.0;      // variable for the proportional gain (response of the system)
float kd = 0.0;      // variable for the derivative gain (undershooting or overshooting)
float ki = 0.0;      // variable for the integral gain (constant error)

void touchSetup()       // function for setting up the touchscreen
{
    stdio_init_all();

    // Setup GPIOs to work with I2C
    gpio_init(SCL);
    gpio_init(SDA);
    gpio_set_pulls(SCL, 1, 0);
    gpio_set_pulls(SDA, 1, 0);

    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_set_function(SDA, GPIO_FUNC_I2C);

    i2c_init(i2c0, 100000); // Initialize i2c0 and set the speed to 100 kb/s standard mode

    uint8_t snd[1] = {0xB0}; // 10110000 pg.25
}

void motorSetup()                                               // function for setting up the servo motors
{
    // SET UP GPIO FOR SERVO NUMBER ONE
    gpio_init(servoOnePin);                                     // initialize the pin connected to servo number one
    gpio_set_dir(servoOnePin, true);                            // set the direction of the pin
    gpio_set_function(servoOnePin, GPIO_FUNC_PWM);              // enable PWM on the pin

    // SET UP GPIO FOR SERVO NUMBER TWO
    gpio_init(servoTwoPin);                                     // initialize the pin connected to servo number one
    gpio_set_dir(servoTwoPin, true);                            // set the direction of the pin
    gpio_set_function(servoTwoPin, GPIO_FUNC_PWM);              // enable PWM on the pin

    // CONFIGURE PWM
    pwm_set_clkdiv_int_frac(7, 50, 0);                          // specify the DIVi and DIVf values
    pwm_set_wrap(7, 37499);                                     // specify the TOP value
    pwm_set_chan_level(7, 0, 0);                                // set the CC value for servo number one to 0
    pwm_set_chan_level(7, 1, 0);                                // set the CC value for servo number one to 0
    pwm_set_enabled(7, true);                                   // enable the pwm signal
}

void readTouchscreenTask()                                      // function to read the current x and y position on the touchscreen
{
    i2c_write_blocking(i2c0, addr, snd, 1, 1); // writes command register to the target
    i2c_write_blocking(i2c0, addr, x_pos, 1, 1);
    i2c_read_blocking(i2c0, addr, x, 2, 0); // reads 2 byte long x value from the target
    i2c_write_blocking(i2c0, addr, snd, 1, 1); // writes command register to the target

    i2c_write_blocking(i2c0, addr, y_pos, 1, 1); // writes command register to the target
    i2c_read_blocking(i2c0, addr, y, 2, 0); // reads 1 byte long y values from target

    // solve for positiion
    xcurrent = (((uint16_t)x[0] << 4) | (x[1] >> 4))*(3.3/4096);
    ycurrent = (((uint16_t)y[0] << 4) | (y[1] >> 4))*(3.3/4096);
}

void controlMotorsTask()        // function to move the servo motors
{
    pwm_set_chan_level(7, 0, xcc);                             // rotate the x axis servo 
    pwm_set_chan_level(7, 1, ycc);                             // rotate the y axis servo
}

int main()
{
    touchSetup();                                                   // call the touchscreen setup function

    motorSetup();                                                   // call the motor setup function

    while (true)
    {
        uint currenttime = time_us_32();        // variable for the current clock time

        if ((currenttime - lasttouchcalled) >= touchdt)     // if 5 ms has passed since the last time the read touchscreen task function was called
        {
            readTouchscreenTask();      // call the read touchscreen task function
            lasttouchcalled = currenttime;      // set the last time the read touchscreen task function was called to the current time
        }

        currentxerror = xdesired - xcurrent;    // calculate the current x axis error
        currentyerror = ydesired - ycurrent;   // calculate the current y axis error

        dxerror = (currentxerror - lastxerror) / dt;     // calculate the derivative of the x axis error
        dyerror = (currentyerror - lastyerror) / dt;     // calculate the derivative of the y axis error

        ixerror += currentxerror * dt;       // calculate the integral of the x axis error
        iyerror += currentyerror * dt;       // calculate the integral of the y axis error

        taux = kp * currentxerror + kd * dxerror + ki * ixerror;       // calculate tau for x
        tauy = kp * currentyerror + kd * dyerror + ki * iyerror;       // calculate tau for y

        // calculate how much to move the x axis
        // calculate how much to move the y axis

        if ((currenttime - lastmotorcalled) >= motordt)     // if 20 ms has passed since the last time the control motors task function was called 
        {
            controlMotorsTask();        // call the control motors task function
            lastmotorcalled = currenttime;      // set the last time the control motors task function was called to the current time
        }

        lastxerror = currentxerror;     // set the last x error to the current x error
        lastyerror = currentyerror;     // set the last y error to the current y error
    }
}