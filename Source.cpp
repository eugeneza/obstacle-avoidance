/******************************************************************************
 * ME 6405 - Final Project
 *
 * Description: Obstacle Avoidance Robot Using 5 sensors
 *
 * Author: Eugene Zaverukha
*******************************************************************************/
/* DriverLib Includes */
#include "msp.h"
#include "driverlib.h"
#include <stdio.h>

void move_forward();
void move_right();
void move_left();
void move_left();
void stop();
void rupt(); // Function to set GPIO Interrupts
void setPWM_5_6(int cycle);

int counter = 0;
int dc = 24000;

int velocity = 343; // Speed of Sound in m/s
int d = 25; // Threshold distance for robot to start

float volatile distance1 = 0; // Front
float volatile distance2 = 0; // Forward left
float volatile distance3 = 0; // Forward right
float volatile distance4 = 0; // Back left
float volatile distance5 = 0; // Back right

float volatile echod = 0; //Echo Time elapsed for respective sensor
int volatile echo = 0; //Echo Timer Start for respective sensor
int volatile edge = 0; // Falling edge is 0, rising edge is 1 for respective sensor

int volatile time = 0; // Keep a timer in seconds
int volatile t = 0; // Keep a general timer

int mode; // 1 is forward moving, 2 is side viewing

const Timer_A_UpModeConfig upConfig_0 =     // Configure counter in Up mode
{ TIMER_A_CLOCKSOURCE_SMCLK,              // Tie Timer A to SMCLK
    TIMER_A_CLOCKSOURCE_DIVIDER_1,         // Increment counter every 64 clock cycles
    480,                                  // Period of Timer A (this value placed in TAxCCR0)
    TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer A rollover interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,     // Enable Capture Compare interrupt
    TIMER_A_DO_CLEAR                        // Clear counter upon initialization};
};

int main(void) {
    WDT_A_holdTimer(); // Turn watchdog off
    unsigned int dcoFrequency = 24E6;
    MAP_CS_setDCOFrequency(dcoFrequency);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    //setting up pwm on pin 5.6
    setPWM_5_6(dc);

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6); // Motor 1
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN7); // Motor 1
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0); // Motor 2
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1); // Motor 2
    //setting initial state off
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); // Motor 1
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); // Motor 1
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Motor 2
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1); // Motor 2

    //// Set Up Trigger Pins
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);     //// Trigger Pin is Output (P4.1) //// For US1
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN2);     //// Trigger Pin is Output (P4.2) //// For US2
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN3);     //// Trigger Pin is Output (P4.3) //// For US3
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN3);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN4);     //// Trigger Pin is Output (P4.4) //// For US4
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN5);     //// Trigger Pin is Output (P4.5) //// For US4
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5);

    rupt(); // Initialize All Interrupts

    while (1) {
        if (time > 1) {
            while ((distance1 >= d) && (distance2 >= d) && (distance3 >= d)) { // Forward if no obstacles within threshold
                mode = 1;
                move_forward();
            }
            stop(); // Comes to a stop
            mode = 2; // Set to side viewing mode
            if (distance4 > distance5) { // Check which way to go (4 is sensor on left, 5 is on right)
                while ((distance1 < d) || (distance2 < d) || (distance3 < d)) {
                    mode = 2; // Set to side viewing mode
                    mode = 1; // Set to forward moving mode
                    move_left();
                }
            }
            if (distance4 <= distance5) {
                while ((distance1 < d) || (distance2 < d) || (distance3 < d)) {
                    mode = 2; // Set to side viewing mode
                    mode = 1; // Set to forward moving mode
                    move_right();
                }
            }
            stop(); // Comes to a stop
        }
    }
}

void TA0_0_IRQHandler(void) {
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    ++t; // Increment t to count every 20 us

    if (t % 5000 == 0) { // Triggers trigger pin once every second // IDEALLY THIS IF STATEMENT SHOULD BE INSIDE WHILE LOOP
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1); // Set Trigger Pin to high for US1
        ++time; // Increment actual timer in seconds
    }

    if (t % 5001 == 0) {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1); // Set Trigger Pin to Low if it's already High
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN4, GPIO_LOW_TO_HIGH_TRANSITION); // Set GPIO interrupt to rising edge for US1
    }

    if ((t % 6000 == 0) && !(t % 5000 == 0) && !(t % 7000 == 0) && mode == 1) {//US2
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2); // Set Trigger Pin to high for US2
    }

    if ((t % 6000 == 0) && !(t % 5000 == 0) && !(t % 7000 == 0) && mode == 2) {//US4
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN4); // Set Trigger Pin to high for US4
    }

    if ((t % 6001 == 0) && !(t % 5001 == 0) && !(t % 7001 == 0) && mode == 1) {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2); // Set Trigger Pin to Low if it's already High
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN6, GPIO_LOW_TO_HIGH_TRANSITION); // Set GPIO interrupt to rising edge for US2
    }

    if ((t % 6001 == 0) && !(t % 5001 == 0) && !(t % 7001 == 0) && mode == 2) { //US4
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN4); // Set Trigger Pin to Low if it's already High
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P2, GPIO_PIN5, GPIO_LOW_TO_HIGH_TRANSITION); // Set GPIO interrupt to rising edge for US1
    }

    if ((t % 7000 == 0) && !(t % 5000 == 0) && mode == 1) { //US3
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN3); // Set Trigger Pin to high for US3
    }

    if ((t % 7000 == 0) && !(t % 5000 == 0) && mode == 2) { //US5
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5); // Set Trigger Pin to high for US5
    }

    if ((t % 7001 == 0) && !(t % 5001 == 0) && mode == 1) { //US3
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN3); // Set Trigger Pin to Low if it's already High
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN4, GPIO_LOW_TO_HIGH_TRANSITION); // Set GPIO interrupt to rising edge for US3
    }

    if ((t % 7001 == 0) && !(t % 5001 == 0) && mode == 2) { // US5
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5); // Set Trigger Pin to Low if it's already High
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P3, GPIO_PIN2, GPIO_LOW_TO_HIGH_TRANSITION); // Set GPIO interrupt to rising edge for US5
    }
}

void PORT2_IRQHandler(void) {
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, status);
    if (edge == 0) { // If interrupt is falling edge its time to take measurement
        echod = (t - echo) * 0.00002; // Get Echo pin time width in seconds (0.0002 is just conversion into seconds)
        distance4 = (velocity * echod) * 50; // Get distance of object in cm and divide by 2 (m/s * s *cm/m)
        edge = 1;
    }

    else {
        echo = t; // Capture time at which Echo pin became active
        edge = 0;
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P2, GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION); // Set GPIO interrupt to falling edge
    }
}

void PORT3_IRQHandler(void) {
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, status);
    if (edge == 0) { // If interrupt is falling edge its time to take measurement
        echod = (t - echo) * 0.00002; // Get Echo pin time width in seconds (0.0002 is just conversion into seconds)
        distance5 = (velocity * echod) * 50; // Get distance of object in cm and divide by 2 (m/s * s *cm/m)
        edge = 1;
    }

    else {
        echo = t; // Capture time at which Echo pin became active
        edge = 0;
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P3, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION); // Set GPIO interrupt to falling edge
    }
}

void PORT4_IRQHandler(void) {
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, status);
    if (edge == 0) { // If interrupt is falling edge its time to take measurement
        echod = (t - echo) * 0.00002; // Get Echo pin time width in seconds (0.0002 is just conversion into seconds)
        distance2 = (velocity * echod) * 50; // Get distance of object in cm and divide by 2 (m/s * s *cm/m)
        edge = 1;
    }

    else {
        echo = t; // Capture time at which Echo pin became active
        edge = 0;
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION); // Set GPIO interrupt to falling edge
    }
}

void PORT5_IRQHandler(void) {
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);
    if (edge == 0) { // If interrupt is falling edge its time to take measurement
        echod = (t - echo) * 0.00002; // Get Echo pin time width in seconds (0.0002 is just conversion into seconds)
        distance3 = (velocity * echod) * 50; // Get distance of object in cm and divide by 2 (m/s * s *cm/m)
        edge = 1;
    }

    else {
        echo = t; // Capture time at which Echo pin became active
        edge = 0;
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P5, GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION); // Set GPIO interrupt to falling edge
    }
}

void PORT6_IRQHandler(void) {
    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P6);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P6, status);
    if (edge == 0) { // If interrupt is falling edge its time to take measurement
        echod = (t - echo) * 0.00002; // Get Echo pin time width in seconds (0.0002 is just conversion into seconds)
        distance1 = (velocity * echod) * 50; // Get distance of object in cm and divide by 2 (m/s * s *cm/m)
        edge = 1;
    }
    else {
        echo = t; // Capture time at which Echo pin became active
        edge = 0;
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P6, GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION); // Set GPIO interrupt to falling edge
    }
}

// FUNCTIONS FOR ACTUATING THE MOTORS
void move_forward() { // Moves Robot Forward
    TA2CCR1 = dc * .5;
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); // Motor 1 Fwd
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Motor 2 Fwd
}

void move_right() { // Moves Robot Right
    TA2CCR1 = dc * .3;
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); // Motor 1 Fwd
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); // Motor 2 Back
}

void move_left() { // Moves Robot Left
    TA2CCR1 = dc * .3;
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); // Motor 1 Back
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Motor 2 Fwd
}

void stop() { // Stops Robot
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); // Motor 1 Low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); // Motor 2 Low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
    int i = 0;
    for (i = 0; i < 500000; ++i) {
    }
}

void rupt() {
    //// Configure Echo Pin (P2.5) as an input and enable GPIO interrupt //// US 4
    MAP_GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN5);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, GPIO_PIN5);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN5);
    MAP_Interrupt_enableInterrupt(INT_PORT2);
    Interrupt_setPriority(INT_PORT2, 5); // Set to highest priority

    //// Configure Echo Pin (P3.2) as an input and enable GPIO interrupt //// US 5
    MAP_GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN2);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN2);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN2);
    MAP_Interrupt_enableInterrupt(INT_PORT3);
    Interrupt_setPriority(INT_PORT3, 6); // Set to highest priority

    //// Configure Echo Pin (P4.6) as an input and enable GPIO interrupt //// US 2
    MAP_GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN6);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN6);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN6);
    MAP_Interrupt_enableInterrupt(INT_PORT4);
    Interrupt_setPriority(INT_PORT4, 6); // Set to highest priority

    //// Configure Echo Pin (P5.4) as an input and enable GPIO interrupt //// US 3
    MAP_GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN4);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN4);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN4);
    MAP_Interrupt_enableInterrupt(INT_PORT5);
    Interrupt_setPriority(INT_PORT5, 5); // Set to highest priority

    //// Configure Echo Pin (P6.4) as an input and enable GPIO interrupt //// US 1
    MAP_GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN4);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P6, GPIO_PIN4);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN4);
    MAP_Interrupt_enableInterrupt(INT_PORT6);
    Interrupt_setPriority(INT_PORT6, 7); // Set to highest priority

    //// Configure Timer A Interrupt ////
    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig_0); // Configure Timer A using above interrupt
    MAP_Interrupt_enableInterrupt(INT_TA0_0); // Enable Timer A interrupt
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE); // Start Time A
   // MAP_Interrupt_enableSleepOnIsrExit();
    Interrupt_enableMaster(); //// Enable Master ////
}

void setPWM_5_6(int cycle) {
    P5SEL0 |= 0x40;// Set     4 of P2SEL0 to enable TA2.1 functionality on P5.6
    P5SEL1 &= ~0x40;// Clear bit 4 of P2SEL1 to enable TA2.1 functionality on P5.6
    P5DIR |= 0x40;// Set pin 5.6 as an output pin

    // Set Timer A period (PWM signal period)
    TA2CCR0 = 24000;
    // Set Duty cycle 0 initially
    TA2CCR1 = dc;
    // Set output mode to Reset/Set
    TA2CCTL1 = OUTMOD_7;    // Macro which is equal to 0x00e0, defined in msp432p401r.h
    // Initialize Timer A
    TA2CTL = TASSEL__SMCLK | MC__UP | TACLR;// Tie Timer A to SMCLK, use Up mode, and clear TA0R
}