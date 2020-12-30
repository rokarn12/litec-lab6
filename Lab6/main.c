// Lab 6
// Rojan Karn

////////////////////////////////////////////////////////////////////
// THIS LAB DEMONSTRATES A PROPORTIONAL-DERIVATIVE CONTROL SCHEME //
// - Controls a gondola to follow a desired heading using a tail  //
//   fan whose pulsewidth is controlled using proportional and    //
//   derviative control methods.                                  //
////////////////////////////////////////////////////////////////////

// This lab uses the Proportional-Derivative control equation to make the gondola
// follow the desired heading coming from a separate compass

// Heading, Desired Heading, and FAN_PW printed along with ranger and compass readings every 100 ms

#define RIN 661887480
#include "C8051_SIM.h" // simulator header file
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void PCA_Init (void);
void Interrupt_Init(void);
void XBR0_Init();
void SMB_Init();
void ReadCompass();
void Set_FAN_Pulsewidth(void);
void getDesiredHeading(void);
void followTicker(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
// Global variables for the important Fan pulse widths
signed int FAN_CENTER = 2765; // 1.5 ms
signed int FAN_MIN = 2028; // 1.1 ms
signed int FAN_MAX = 3502; // 1.9 ms
signed int FAN_PW = 0;

// use for 1 second delay in the beginning
unsigned char PCA_counts;

unsigned char Data[2] = {0, 0, 0};        // Data array for use with the SMB/I2C
uint8_t h_count = 0;        // Overflow tracker for the heading measurement
uint8_t new_heading = 0;    // Flag to denote time to read heading
uint16_t heading = 0;       // Value of heading
uint8_t new_print = 0;      // Flag to denote time to print
uint8_t p_count = 0;        // Overflow tracker for printing

// Error values
signed int heading_error, target_heading, derivative, prev_error = 0;
float proportional_control_FAN = 1.5; // proportional gain constant
float derivative_constant = 70; // derivative gain constant


int main()
{
    // Initializations
    Sys_Init();
    Port_Init();
    XBR0_Init();
    PCA_Init();
    Interrupt_Init();
    SMB_Init();

    // set FAN Pulsewidth to Neutral
    FAN_PW = FAN_CENTER;
    PCA0CP0 = 0xFFFF - FAN_PW;

    // wait 1 second
    PCA_counts = 0;
    while (PCA_counts < 50) Sim_Update();

    // main loop
    while(1){
        Sim_Update();
        if(new_heading) { // if it is time for a new compass reading
            new_heading = 0; // clear heading flag

            // call main driver function
            followTicker();
        }
        else if (new_print) { // if it is time to print the measurement
            new_print = 0;
            printf("HEADING: %d \t DESIRED HEADING: %d \t FAN_PW: %d\r\n", heading, target_heading, FAN_PW); // print statement
        }
    }
}

//-----------------------------------------------------------------------------
// followTicker
//-----------------------------------------------------------------------------
//
// Use readings from the compasses as well as the PD control equation to adjust
// the Fan's pulsewidth so that it can follow the blue ticker
//
void followTicker(void) {
    ReadCompass(); // call helper function
    getDesiredHeading(); // call other function to get the desired heading

    // calculate the heading error
    heading_error = target_heading - heading;

    // adjust error measurement
    if (heading_error > 1800) {
        heading_error -= 3600;
    } else if (heading_error < -1800) {
        heading_error += 3600;
    }

    // calculate the derivative value
    derivative = heading_error - prev_error;

    // PD control equation using the constants kp and kd
    FAN_PW = FAN_CENTER + (proportional_control_FAN * heading_error) + (derivative_constant * derivative);

    // assign the current error as the previous one for the next iteration
    prev_error = heading_error;

    // update the Fan's pulsewidth
    Set_FAN_Pulsewidth();
}

//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
//
// Set up ports for input and output
//
void Port_Init()
{
    P0MDOUT |= 0x30;  //set output pin for CEX0 or CEX2 in push-pull mode

    // port 3
    P3MDOUT &= 0x1F;
    P3 |= ~0x1F;
}

//-----------------------------------------------------------------------------
// XBR0_Init
//-----------------------------------------------------------------------------
//
// Set up the crossbar
//
void XBR0_Init()
{

    XBR0 |= 0x1D;  //configure crossbar as directed in the laboratory
    XBR0 &= ~0x20;

}

//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
//
// Set up Programmable Counter Array
//
void PCA_Init(void)
{
    PCA0MD &= 0xF0;  // SYSCLK/12
    PCA0MD |= 0x01;

    // Enable 16-bit PWM CCMs
    PCA0CPM0 = 0xC2;
    PCA0CPM1 = 0xC2;
    PCA0CPM2 = 0xC2;
    PCA0CPM3 = 0xC2;
    PCA0CPM4 = 0xC2;

    // Turn PCA on
    PCA0CN = 0x40;
}

//-----------------------------------------------------------------------------
// Interrupt_Init
//-----------------------------------------------------------------------------
//
// Set up the PCA overflow interrupts
//
void Interrupt_Init()
{
    EA = 1;
    EIE1 |= 0x08;
}

//-----------------------------------------------------------------------------
// SMB_Init
//-----------------------------------------------------------------------------
//
// Initialize the SMBus
//
void SMB_Init()
{
    // Set the clock rate of the SMBus/I2C and
    // Enable the SMBus/I2C (check manual)
    ENSMB = 1;
    SMB0CR = 0x93;
}

//-----------------------------------------------------------------------------
// ReadCompass
//-----------------------------------------------------------------------------
//
// Read the heading value from the compass
//
void ReadCompass(void){
    i2c_read_data(0xC0, 2, Data, 2);   // Read the 0-3600 heading bytes
    heading = (Data[0] << 8) | Data[1];                  // Put the bytes together
}

//-----------------------------------------------------------------------------
// getDesiredHeading
//-----------------------------------------------------------------------------
//
// Read a different compass to retrieve the desired heading
//
void getDesiredHeading(void) {
    i2c_read_data(0x42, 2, Data, 2);   // Read the 0-3600 heading bytes (from address 0x42)
    target_heading = (Data[0] << 8) | Data[1];                  // Put the bytes together
    //target_heading = 2750;
}

//-----------------------------------------------------------------------------
// Set_FAN_Pulsewidth
//-----------------------------------------------------------------------------
//
// Check the boundaries of the pulsewidth then assign it to CEX0
//
void Set_FAN_Pulsewidth(void)
{
    // check limits
    if (FAN_PW > FAN_MAX) {
        FAN_PW = FAN_MAX;
    } else if (FAN_PW < FAN_MIN) {
        FAN_PW = FAN_MIN;
    }

    // Assign FAN_PW to the PCA
    PCA0CP0 = 0xFFFF - FAN_PW;
}


//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//
void PCA_ISR ( void )
{
    if(CF){
        CF = 0;
        PCA0 = 0xFFFF - 36863;
        // increment all counters
        PCA_counts++;
        h_count++;
        p_count++;

        if(h_count == 2){  // Count 40 ms
            h_count = 0;
            new_heading = 1;
        }
        if (p_count == 5) { // Count 100 ms
            p_count = 0;
            new_print = 1;
        }
    }
    PCA0CN &= 0x40;
}

// By: Rojan Karn
