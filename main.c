#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/timer.h>
#include <hardware/i2c.h>

// PINS
#define SCL 1 // Define GPIO for i2c0 SCL Line
#define SDA 0 // Define GPIO for i2c0 SDA Line
#define servoOnePin 15
#define servoTwoPin 10

// TOUCHSCREEN
#define addr 0x48 // touchscreen target address both A0 and A1 are held low making address 1001000
const int Lx = 165;
const int Ly = 105;
float xcurrent = 0;  // creates a variable that reads x position
float ycurrent = 0;  // creates a variable that reads y position
uint8_t snd[1] = {0xB0}; // 10110000 pg.25

// SERVO
uint16_t ccOne = 0;
uint16_t ccTwo = 0;
uint8_t sliceOne;
uint8_t sliceTwo;
uint8_t channelOne;
uint8_t channelTwo;
uint8_t divI = 50;
uint8_t divF = 0;
uint16_t top = 37499;

// TIMING
const uint touchdt = 5000;      // variable for how often the read touchscreen task should be executed
const uint motordt = 20000;     // variable for how often the control motors task should be executed
uint lasttouchcalled = 0;       // variable for the last time the read touchscreen task was executed
uint lastmotorcalled = 0;       // variable for the last time the control motors task was executed

// PID VARIABLES
float dt = 0.0;              // variable for the differential amount of time
float dxerror = 0.0;         // variable for the derivative of the x axis error
float dyerror = 0.0;         // variable for the derivative of the y axis error

float xdesired = 1.65;    // variable for the desired x axis position of the ball
float ydesired = 1.65;    // variable for the desired y axis position of the ball
float currentxerror = 0.0;       // variable for the current x axis error (distance between the desired position and the actual position)
float currentyerror = 0.0;       // variable for the curent y axis error (distance between the desired position and the actual position)
float lastxerror = 0.0;      // variable for the last x axis error (distance between the desired position and the actual position)
float lastyerror = 0.0;      // variable for the last y axis error (distance between the desired position and the actual position)
float taux = 0.0;           // variable for tau x
float tauy = 0.0;           // variable for tau y


void touchscreenSetup()
{
    // Setup GPIOs to work with I2C
    gpio_init(SCL);
    gpio_init(SDA);
    gpio_set_pulls(SCL, 1, 0);
    gpio_set_pulls(SDA, 1, 0);

    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_set_function(SDA, GPIO_FUNC_I2C);

    i2c_init(i2c0, 100000); // Initialize i2c0 and set the speed to 100 kb/s standard mode
}

void motorSetup()
{
    gpio_init(servoOnePin);
    gpio_set_dir(servoOnePin, true);
    gpio_set_function(servoOnePin, GPIO_FUNC_PWM);

    sliceOne = pwm_gpio_to_slice_num(servoOnePin);
    channelOne = pwm_gpio_to_channel(servoOnePin);

    pwm_set_clkdiv_int_frac(sliceOne, divI, divF);
    pwm_set_wrap(sliceOne, top);
    pwm_set_chan_level(sliceOne, channelOne, 3750);
    pwm_set_enabled(sliceOne, true);

    gpio_init(servoTwoPin);
    gpio_set_dir(servoTwoPin, true);
    gpio_set_function(servoTwoPin, GPIO_FUNC_PWM);

    sliceTwo = pwm_gpio_to_slice_num(servoTwoPin);
    channelTwo = pwm_gpio_to_channel(servoTwoPin);

    pwm_set_clkdiv_int_frac(sliceTwo, divI, divF);
    pwm_set_wrap(sliceTwo, top);
    pwm_set_chan_level(sliceTwo, channelTwo, 3750);
    pwm_set_enabled(sliceTwo, true);
}

void readTouchscreenTask()                                      // function to read the current x and y position on the touchscreen
{   
    uint8_t x_pos[1] = {0xC0};
    uint8_t y_pos[1] = {0xD0};
    uint8_t x[2] = {0, 0};
    uint8_t y[2] = {0, 0};

//    i2c_write_blocking(i2c0, addr, snd, 1, 1);          // writes command register
    i2c_write_blocking(i2c0, addr, x_pos, 1, 1);
    i2c_read_blocking(i2c0, addr, x, 2, 0); // reads 2 byte long x value from the target

    i2c_write_blocking(i2c0, addr, y_pos, 1, 1); // writes command register to the target
    i2c_read_blocking(i2c0, addr, y, 2, 0); // reads 1 byte long y values from target
    
    xcurrent = (((uint16_t)x[0] << 4) | (x[1] >> 4))*(3.3/4096); 
    ycurrent = (((uint16_t)y[0] << 4) | (y[1] >> 4))*(3.3/4096);

    if ( xcurrent < 0.01 || xcurrent > 3.29 )
    {
        xcurrent = 1.65;
    }

    if ( ycurrent < 0.01 || ycurrent > 3.29 )
    {
        ycurrent = 1.65;
    }
    //printf("X position: %f\tY position: %f\n", xcurrent, ycurrent);
}

void pidCalculation()
{
    float kp = 10.0;      // variable for the proportional gain (response of the system)
    float kd = 0.0;      // variable for the derivative gain (undershooting or overshooting)
    float ki = 0.0;      // variable for the integral gain (constant error)
    static float ixerror = 0.0;         // variable for the integral of the x axis error
    static float iyerror = 0.0;         // variable for the integral of the y axis error
    
    dt = touchdt;

    currentxerror = xdesired - xcurrent;    // calculate the current x axis error
    currentyerror = ydesired - ycurrent;   // calculate the current y axis error

    //printf("Current X Error: %f\tCurrent Y Error: %f\n", currentxerror, currentyerror);

    dxerror = (currentxerror - lastxerror) / dt;     // calculate the derivative of the x axis error
    dyerror = (currentyerror - lastyerror) / dt;     // calculate the derivative of the y axis error

    //printf("dx Error: %f\tdy Error: %f\n", dxerror, dyerror);

    ixerror += currentxerror * dt;       // calculate the integral of the x axis error
    iyerror += currentyerror * dt;       // calculate the integral of the y axis error

    //printf("ix Error: %f\tiy Error: %f\n", ixerror, iyerror);

    taux = kp * currentxerror + kd * dxerror + ki * ixerror;       // calculate tau for x
    tauy = kp * currentyerror + kd * dyerror + ki * iyerror;       // calculate tau for y

    //printf("Proportional X: %f\tProportional Y: %f\n", (kp * currentxerror), (kp * currentyerror));

    printf("taux: %f\ttauy: %f\n", taux, tauy);

    lastxerror = currentxerror;     // set the last x error to the current x error
    lastyerror = currentyerror;     // set the last y error to the current y error
}

void motorControlTask()
{ 
    //uint16_t ccOne = (uint16_t) ((1363 * taux) + 1500);    // 180* degrees of rotation (54.5454* for 1 x value -or- 1363 cc for 1 x value)
    //uint16_t ccTwo = (uint16_t) ((1363 * tauy) + 1500);    // 180* degress of rotation (54.5454* for 1 y value -or- 1363 cc for 1 y value)

    //uint16_t ccOne = (uint16_t) ((681.81 * taux) + 2625);    // 90* degrees of rotation (27.2727* for 1 x value -or- 681.81 cc for 1 x value)
    //uint16_t ccTwo = (uint16_t) ((681.81 * tauy) + 2625);    // 90* degress of rotation (27.2727* for 1 y value -or- 681.81 cc for 1 y value)

    uint16_t ccOne = (uint16_t) ((340.9 * taux) + 3187.5);    // 45* degrees of rotation (13.6363* for 1 x value -or- 340.90 cc for 1 x value)
    uint16_t ccTwo = (uint16_t) ((340.9 * tauy) + 3187.5);    // 45* degress of rotation (13.6363* for 1 y value -or- 340.90 cc for 1 y value)

    //printf("ccOne: %d\tccTwo: %d\n", ccOne, ccTwo);

    pwm_set_chan_level(sliceOne, channelOne, ccOne);
    pwm_set_chan_level(sliceTwo, channelTwo, ccTwo);
}

int main()
{
    stdio_init_all();

    touchscreenSetup();

    motorSetup();

    while (true)
    {
        uint currenttime = time_us_32();        // variable for the current clock time

        if ((currenttime - lasttouchcalled) >= touchdt)     // if 5 ms has passed since the last time the read touchscreen task function was called
        {
            readTouchscreenTask();      // call the read touchscreen task function
            lasttouchcalled = currenttime;      // set the last time the read touchscreen task function was called to the current time
        }

        if ((currenttime - lastmotorcalled) >= motordt)     // if 20 ms has passed since the last time the control motors task function was called 
        {
            pidCalculation();
            motorControlTask();        // call the control motors task function
            lastmotorcalled = currenttime;      // set the last time the control motors task function was called to the current time
        }
         
    }
}