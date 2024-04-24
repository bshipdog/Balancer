#include <stdio.h>
#include <pico/stdlib.h>
#include <stdlib.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/timer.h>
#include <hardware/i2c.h>
#include <hardware/uart.h>

// PINS
#define SCL 1                                               // Define GPIO for i2c0 SCL Line
#define SDA 0                                               // Define GPIO for i2c0 SDA Line
#define servoOnePin 15                                      // gpio for the x axis servo
#define servoTwoPin 10                                      // gpio for the y axis servo
#define TX 16                                               // gpio pin for the bt module
#define RX 17                                               // gpio pin for the bt module

// BLUETOOTH
float xdesired = 82.5;                                         // variable for the desired x axis position of the ball
float ydesired = 52.5;                                         // variable for the desired y axis position of the ball

// TOUCHSCREEN
#define addr 0x48                                           // touchscreen target address both A0 and A1 are held low making address 1001000
const int Lx = 165;
const int Ly = 105;
const double o_x = Lx/2;                                    // x origin
const double o_y = Ly/2;                                    // y origin
float xcurrent = 0;                                         // creates a variable that reads x position
float ycurrent = 0;                                         // creates a variable that reads y position
float zcurrent = 0;                                         // creates a variable that reads z position
uint8_t x_pos[1] = {0xC0};
uint8_t y_pos[1] = {0xD0};
uint8_t z_pos[1] = {0xE0};                                  // for z1
uint8_t x[2] = {0, 0};
uint8_t y[2] = {0, 0};
uint8_t z[2] = {0, 0};

// SERVO
uint16_t ccOne = 0;                                         // variable for the x axis cc value
uint16_t ccTwo = 0;                                         // variable for the y axis the cc value
uint8_t sliceOne;                                           // slice for the x axis servo
uint8_t sliceTwo;                                           // slice for the y axis servo
uint8_t channelOne;                                         // channel for the x axis servo
uint8_t channelTwo;                                         // channel for the y axis servo
uint8_t divI = 50;                                          // divI value
uint8_t divF = 0;                                           // divF value
uint16_t top = 37499;                                       // top value

// CHANGE THE VALUES BELOW TO LEVEL THE TOUCHSCREEN
uint16_t flatX = 3750;                                      // variable for leveling the x axis
uint16_t flatY = 3750;                                      // variable for leveling the y axis

// 25 cc per degree
uint16_t mapX = 25;                              // variable for translating degrees of rotation to cc for the x axis
// 25 cc per degree
uint16_t mapY = 25;                              // variable for translating degrees of rotation to cc for the y axis


// TIMING
const uint touchdt = 5000;                                  // variable for how often the read touchscreen task should be executed
const uint motordt = 20000;                                 // variable for how often the control motors task should be executed
const uint promptdt = 1000000;
uint lasttouchcalled = 0;                                   // variable for the last time the read touchscreen task was executed
uint lastmotorcalled = 0;                                   // variable for the last time the control motors task was executed
uint currenttime = 0;                                       // current time variable
uint lastprompt = 0;

// PID VARIABLES
float dt = 0.0;                                             // variable for the differential amount of time
float dxerror = 0.0;                                        // variable for the derivative of the x axis error
float dyerror = 0.0;                                        // variable for the derivative of the y axis error

float currentxerror = 0.0;                                  // variable for the current x axis error (distance between the desired position and the actual position)
float currentyerror = 0.0;                                  // variable for the curent y axis error (distance between the desired position and the actual position)
float lastxerror = 0.0;                                     // variable for the last x axis error (distance between the desired position and the actual position)
float lastyerror = 0.0;                                     // variable for the last y axis error (distance between the desired position and the actual position)
float taux = 0.0;                                           // variable for tau x
float tauy = 0.0;                                           // variable for tau y

// SET UP GPIO FOR BT MODULE COMMUNICATION
void btSetup()
{
    gpio_init(TX);
    gpio_init(RX);
    gpio_set_dir(TX, true);
    gpio_set_dir(RX, false);
    gpio_set_function(TX, GPIO_FUNC_UART);
    gpio_set_function(RX, GPIO_FUNC_UART);

    uart_init(uart0, 38400);        // set the baud rate to 38400

    sleep_ms(5000);     // wait while the bt module powers up into configuration mode

    /* CONFIGURE THE BT MODULE
    for (int i=0; i<10; i++){
        printf("Sending AT command to bluetooth module!\r\n");
        uart_puts(uart0, "AT\r\n");
        getMessage();

        sleep_ms(5000);
    }

    //factoryReset();     // call the factory resest function
    //getMessage();       // call the get message function
    //sleep_ms(5000);     // wait for the changes to go into effect  
    
    /*changeName();       // call the change name function
    getMessage();       // call the get message function
    sleep_ms(5000);     // wait for the changes to go into effect

    changePassword();       // call the change password function
    getMessage();           // call the get message function
    sleep_ms(5000);         // wait for the changes to go into effect

    changeUART();           // call the change uart function
    getMessage();           // call the get message function
    sleep_ms(5000);         // wait for the changes to go into effect*/
}

void getMessage()
{
    while(uart_is_readable_within_us(uart0, 1000000))
    {
        char c = uart_getc(uart0);
        printf("%c", c);
    }
}

// RESET THE BT MODULE TO FACTORY SETTINGS
void factoryReset()
{
    printf("Sending AT+ORGL command!");
    uart_puts(uart0, "AT+ORGL\r\n");        // the AT+ORGL command resets the bt module to original settings
}

// CHANGE THE NAME OF THE BT MODULE
void changeName()
{
    printf("Sending AT+NAME command!");
    uart_puts(uart0, "AT+NAME=BallBalancer\r\n");        // the AT+NAME=Ball Balancer command changes the name of the bt module to Ball Balancer
}

// CHANGE THE PASSWORD OF THE BT MODULE
void changePassword()
{
    printf("Sending AT+PSWD command!");
    uart_puts(uart0, "AT+PSWD=3185\r\n");        // the AT+PSWD=5813 command changes the password of the bt module to 5813
}

// SET THE UART PARAMETERS OF THE BT MODULE
void changeUART()
{
    printf("Sending AT+UART command!");
    uart_puts(uart0, "AT+UART=38400,0,0\r\n");        // the AT+UART=38400,0,0 command changes the uart parameters of the bt module to a 38400 baud rate, a single stop bit, and no parity bit
}

void touchscreenSetup()
{
    // Setup GPIOs to work with I2C
    gpio_init(SCL);
    gpio_init(SDA);
    gpio_set_pulls(SCL, 1, 0);
    gpio_set_pulls(SDA, 1, 0);

    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_set_function(SDA, GPIO_FUNC_I2C);

    i2c_init(i2c0, 100000);                                                     // Initialize i2c0 and set the speed to 100 kb/s standard mode
}

// SET UP GPIO FOR SERVOS AND START PWM
void motorSetup()
{
    gpio_init(servoOnePin);
    gpio_set_dir(servoOnePin, true);
    gpio_set_function(servoOnePin, GPIO_FUNC_PWM);

    sliceOne = pwm_gpio_to_slice_num(servoOnePin);
    channelOne = pwm_gpio_to_channel(servoOnePin);

    pwm_set_clkdiv_int_frac(sliceOne, divI, divF);
    pwm_set_wrap(sliceOne, top);
    pwm_set_chan_level(sliceOne, channelOne, flatX);                            // set the x axis servo to the flat position
    pwm_set_enabled(sliceOne, true);

    gpio_init(servoTwoPin);
    gpio_set_dir(servoTwoPin, true);
    gpio_set_function(servoTwoPin, GPIO_FUNC_PWM);

    sliceTwo = pwm_gpio_to_slice_num(servoTwoPin);
    channelTwo = pwm_gpio_to_channel(servoTwoPin);

    pwm_set_clkdiv_int_frac(sliceTwo, divI, divF);
    pwm_set_wrap(sliceTwo, top);
    pwm_set_chan_level(sliceTwo, channelTwo, flatY);                            // set the y axis servo to the flat position
    pwm_set_enabled(sliceTwo, true);
}

// GET A MESSAGE FROM THE BT MODULE
void prompt()
{
    while(uart_is_readable_within_us(uart0, 1000))
    {
        uart_puts(uart0, "\r\nInput desired position: ");
        char c = uart_getc(uart0);                                              // variable for storing the character received from the bt module
        if(c == 'X'){                                                           // if the first character received is x
            char xvalue[3];                                                     // define a character array for the x distance
            xvalue[0] = uart_getc(uart0);                                       // put the first distance character received into the arrays first position
            xvalue[1] = uart_getc(uart0);                                       // put the second distance character received into the arrays second position
            xvalue[2] = uart_getc(uart0);                                       // put the third distance character received into the arrays third position
            xdesired = atof(xvalue);
            
            if (xdesired < 5 || xdesired > 160)
            {
                uart_puts(uart0, "\r\nTry again: ");
                xdesired = 82.5;
            }                                            // convert the character array into an integer
        }
        else if (c == 'Y')
        {                                                    // if the first character received is y
            char yvalue[3];                                                     // define a character array for the y distance
            yvalue[0] = uart_getc(uart0);                                       // put the first distance character received into the arrays first position
            yvalue[1] = uart_getc(uart0);                                       // put the second distance character received into the arrays second position
            yvalue[2] = uart_getc(uart0);                                       // put the third distance character received into the arrays third position
            ydesired = atof(yvalue);                                            // convert the character array into an integer

            if (ydesired < 5 || ydesired > 100)
            {
                uart_puts(uart0, "\r\nTry again: ");
                ydesired = 52.5;
            }
        }
    }
}

void readTouchscreenTask()                                                      // function to read the current x and y position on the touchscreen
{   
// setting to zero
  xcurrent = 0;  
  ycurrent = 0;  
  zcurrent = 0; 

  i2c_write_blocking(i2c0, addr, z_pos, 1, 1);                                  // writes command register to the target
  i2c_read_blocking(i2c0, addr, z, 2, 0);                                       // reads 2 byte long z value from the target
  zcurrent = ((((uint16_t)z[0] << 4) | (z[1] >> 4))*(3.3/4096));

  if (true)                                                           // checks for conditions of whether the screen is touched or not and allows us to determine whether the screen can be set to home position to control motors
  {
    i2c_write_blocking(i2c0, addr, x_pos, 1, 1);                                // writes command register to the target
    i2c_read_blocking(i2c0, addr, x, 2, 0);                                     // reads 2 byte long x value from the target

    i2c_write_blocking(i2c0, addr, y_pos, 1, 1);                                // writes command register to the target
    i2c_read_blocking(i2c0, addr, y, 2, 0);                                     // reads 1 byte long y values from target

      // solve for positiion
    xcurrent = ((((uint16_t)x[0] << 4) | (x[1] >> 4))/4096.0)*Lx;
    ycurrent = ((((uint16_t)y[0] << 4) | (y[1] >> 4))/4096.0)*Ly;
  }

  if (xcurrent == 0.0){
    xcurrent = 82.5;
  }

  if (ycurrent == 0.0){
    ycurrent == 52.5;
  }

  //printf("%.2f\t%.2f\n",  xcurrent, ycurrent); 
}

// FUNCTION FOR CALCULATING THE PID VALUES
void pidCalculation()
{
    float kpX = -0.3;                                                            // variable for the proportional gain (response of the system)
    float kdX = -0.09;                                                            // variable for the derivative gain (undershooting or overshooting)
    float kiX = -0.004;                                                            // variable for the integral gain (constant error)

    float kpY = -0.2;                                                            // variable for the proportional gain (response of the system)
    float kdY = -0.07;                                                            // variable for the derivative gain (undershooting or overshooting)
    float kiY = -0.004;                                                            // variable for the integral gain (constant error)
    
    static float ixerror = 0.0;                                                        // variable for the integral of the x axis error
    static float iyerror = 0.0;                                                        // variable for the integral of the y axis error
    
    dt = .02;

    currentxerror = xdesired - xcurrent;                                        // calculate the current x axis error
    currentyerror = ydesired - ycurrent;                                        // calculate the current y axis error
    //printf("Current X Error: %f\tCurrent Y Error: %f\n", currentxerror, currentyerror);

    dxerror = (currentxerror - lastxerror) / dt;                                // calculate the derivative of the x axis error
    dyerror = (currentyerror - lastyerror) / dt;                                // calculate the derivative of the y axis error
    //printf("dx Error: %f\tdy Error: %f\n", dxerror, dyerror);

    ixerror += currentxerror * dt;                                              // calculate the integral of the x axis error
    iyerror += currentyerror * dt;                                              // calculate the integral of the y axis error
    //printf("ix Error: %f\tiy Error: %f\n", ixerror, iyerror);

    taux = kpX * currentxerror + kdX * dxerror + kiX * ixerror;                 // calculate tau for x
    tauy = kpY * currentyerror + kdY * dyerror + kiY * iyerror;                 // calculate tau for y

    //printf("Proportional X: %f\tProportional Y: %f\n", (kp * currentxerror), (kp * currentyerror));
    //printf("Proportional X: %f\tProportional Y: %f\n", (kdX * dxerror), (kdY * dyerror));

    printf("taux: %f\ttauy: %f\n", taux, tauy);

    lastxerror = currentxerror;                                                 // set the last x error to the current x error
    lastyerror = currentyerror;                                                 // set the last y error to the current y error
}

// FUNCTION FOR CONTROLLING THE MOTORS
void motorControlTask()
{ 

    uint16_t ccOne = (taux * mapX) + flatX;                                     // map taux (horizontal distance) to degrees of rotation and add that value to the flat position                                                              // convert positive tauy value to negative and negative tauy value to positive
    uint16_t ccTwo = (tauy * mapY) + flatY;                                     // map tauy (horizontal distance) to degrees of rotation and add that value to the flat position

    //printf("ccOne: %d\tccTwo: %d\n", ccOne, ccTwo);

    if (xcurrent <= 5.0 || xcurrent >= 160.0){
        ccOne = flatX;
    }

    if (ycurrent <= 5.0 || ycurrent >= 100.0)
    {
        ccTwo = flatY;
    }

    pwm_set_chan_level(sliceOne, channelOne, ccOne);                            // move the x axis servo
    pwm_set_chan_level(sliceTwo, channelTwo, ccTwo);                            // move the y axis servo
}

void main()
{
    stdio_init_all();

    btSetup();                                                                  // call the bt setup function

    touchscreenSetup();                                                         // call the touch screen setup function

    motorSetup();                                                               // call the motor setup function
  
    //sleep_ms(3000);                                                             // sleep for 3 seconds while we get things ready

    while (true){
        uint currenttime = time_us_32();                                        // variable for the current clock time

        if ((currenttime - lasttouchcalled) >= touchdt){                        // if 5 ms has passed since the last time the read touchscreen task function was called
            readTouchscreenTask();                                              // call the read touchscreen task function
            lasttouchcalled = currenttime;                                      // set the last time the read touchscreen task function was called to the current time
        }

        if ((currenttime - lastmotorcalled) >= motordt){                        // if 20 ms has passed since the last time the control motors task function was called 
            if (xcurrent > 5.0 && xcurrent < 160.0 && ycurrent > 5.0 && ycurrent < 100.0)
            {
                taux = 0.0;
                tauy = 0.0;
                pidCalculation();                                                   // call the pid calculation function
            }

            motorControlTask();                                                 // call the control motors task function
            lastmotorcalled = currenttime;                                      // set the last time the control motors task function was called to the current time
        }

        if ((currenttime - lastprompt) >= promptdt)
        {
            prompt();
            lastprompt = currenttime;
        }
    }
}
