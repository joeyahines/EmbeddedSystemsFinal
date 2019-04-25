/*
 * TODO
 * Test collision
 * Tweak treasure find code to be more reliable
 * Test test test
 */

#include "msp.h"
#include "CortexM.h"
#define BASE_POWER 30

enum STATE
{
    STRAIGHT,
    LEFT_TURN,
    RIGHT_TURN,
    FIND_RIGHT_TURN,
    FIND_LEFT_TURN,
    DONE,
    COLLISION,
    TURN_AROUND,
    TURN
};
#define RANGE 40
enum STATE state = STRAIGHT;

//Possible Motor Values
enum motor
{
    Right = BIT6, Left = BIT7
};


//Line Data
int16_t line_data;

//TimerA0 Period
uint16_t PERIOD;

//Clock Globals
uint32_t ClockFrequency = 3000000; // cycles/second

int32_t Prewait = 0; // loops between BSP_Clock_InitFastest() called and PCM idle (expect 0)
uint32_t CPMwait = 0; // loops between Power Active Mode Request and Current Power Mode matching requested mode (expect small)
uint32_t Postwait = 0; // loops between Current Power Mode matching requested mode and PCM module idle (expect about 0)
uint32_t IFlags = 0;                    // non-zero if transition is invalid
uint32_t Crystalstable = 0; // loops before the crystal stabilizes (expect small)

volatile uint8_t bump;
uint8_t bump0, bump1, bump2, bump3, bump4, bump5;
uint8_t Bump_Read();
void (*bump_event)(uint8_t);
void (*line_follower_event)(uint8_t);
void set_left_motor_duty(uint16_t);
void set_right_motor_duty(uint16_t);
void set_motor_power(int8_t power_level, enum motor m);
uint8_t line_follower_read(void);

void delay(unsigned long ulCount)
{
    __asm ( "pdloop:  subs    r0, #1\n"
            "    bne    pdloop\n");
}

// ------------Clock_Delay1ms------------
// Simple delay function which delays about n milliseconds.
// Inputs: n, number of msec to wait
// Outputs: none
void Clock_Delay1ms(uint32_t n)
{
    while (n)
    {
        delay(ClockFrequency / 9162);   // 1 msec, tuned at 48 MHz
        n--;
    }
}

/****************************************
 *  SysTick Timer Init
 ***************************************/
void SysTick_Init(void)
{
    SysTick->CTRL = 0;
    SysTick->LOAD = 6000;           // maximum reload value
    SysTick->VAL = 0;
    SysTick->CTRL = 0x00000007;           // enable SysTick with interrupts
    SCB->SHP[11] = 4 << 5;

}

void SysTick_Wait(uint32_t delay)
{
    SysTick->LOAD = (delay - 1);           // count down to zero
    SysTick->VAL = 0;         // any write to CVR clears it and COUNTFLAG in CSR
    while (( SysTick->CTRL & 0x00010000) == 0)
    {
    };

}
// Time delay using busy wait.
// assumes 48 MHz bus clock
void SysTick_Wait10ms(uint32_t delay)
{
    uint32_t i;
    for (i = 0; i < delay; i++)
    {
        SysTick_Wait(480000);  // wait 10ms (assumes 48 MHz clock)
    }
}

void SysTick_Delay1us(uint32_t delay)
{
    uint32_t i;
    for (i = 0; i < delay / 2; i++)
    {
        SysTick_Wait(2);  // wait 1ms (assumes 48 MHz clock)
    }
}

void Clock_Delay1us(uint32_t n)
{
    n = (382 * n) / 100;

    while (n)
    {
        n--;
    }
}

// delay function
// which delays about 6*ulCount cycles
// ulCount=8000 => 1ms = (8000 loops)*(6 cycles/loop)*(20.83 ns/cycle)
//Code Composer Studio Code
void Clock_Init32KHz(void)
{
    // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
    while (PCM->CTL1 & 0x00000100)
    {
//  while(PCMCTL1&0x00000100){
        Prewait = Prewait + 1;
        if (Prewait >= 100000)
        {
            return;                           // time out error
        }
    }
    // request power active mode LDO VCORE1 to support the 48 MHz frequency
    PCM->CTL0 = (PCM->CTL0 & ~0xFFFF000F) | // clear PCMKEY bit field and AMR bit field
//  PCMCTL0 = (PCMCTL0&~0xFFFF000F) |     // clear PCMKEY bit field and AMR bit field
            0x695A0000 |     // write the proper PCM key to unlock write access
            0x00000001;                 // request power active mode LDO VCORE1
    // check if the transition is invalid (see Figure 7-3 on p344 of datasheet)
    if (PCM->IFG & 0x00000004)
    {
        IFlags = PCM->IFG; // bit 2 set on active mode transition invalid; bits 1-0 are for LPM-related errors; bit 6 is for DC-DC-related error
        PCM->CLRIFG = 0x00000004;           // clear the transition invalid flag
        // to do: look at CPM bit field in PCMCTL0, figure out what mode you're in, and step through the chart to transition to the mode you want
        // or be lazy and do nothing; this should work out of reset at least, but it WILL NOT work if Clock_Int32kHz() or Clock_InitLowPower() has been called
        return;
    }
    // wait for the CPM (Current Power Mode) bit field to reflect a change to active mode LDO VCORE1
    while ((PCM->CTL0 & 0x00003F00) != 0x00000100)
    {
        CPMwait = CPMwait + 1;
        if (CPMwait >= 500000)
        {
            return;                           // time out error
        }
    }
    // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
    while (PCM->CTL1 & 0x00000100)
    {
        Postwait = Postwait + 1;
        if (Postwait >= 100000)
        {
            return;                           // time out error
        }
    }
    // initialize PJ.3 and PJ.2 and make them HFXT (PJ.3 built-in 48 MHz crystal out; PJ.2 built-in 48 MHz crystal in)
    PJ->SEL0 |= 0x03;        // 0000 0011
    PJ->SEL1 &= ~0x03;  // 1111 1100  configure built-in 32 kHz crystal for LFXT

    CS->KEY = 0x695A;            // unlock CS module for register access
    CS->CTL2 = (CS->CTL2 & ~0x00000003) | 0x00000003 | 0x00000100;
    CS->CTL2 &= ~0x00000200;     //disable low-frequency crystal bypass

    // wait for LXFT clock to stabilize
    while (CS->IFG & 0x00000001)
    {
        CS->CLRIFG = 0x00000001;       // clear the LFXT interrupt flag
        Crystalstable = Crystalstable + 1;
        if (Crystalstable > 100000)
            return;            // time out error
    }
    CS->CTL1 = 0x10000200;       // SMCLK/2 HSMCLK MCLK from LXFT
    CS->KEY = 0; // lock CS module                         // lock CS module from unintended access
    ClockFrequency = 48000000;
//  SubsystemFrequency = 12000000;
}

void Clock_Init48MHz(void)
{
    // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
    while (PCM->CTL1 & 0x00000100)
    {
//  while(PCMCTL1&0x00000100){
        Prewait = Prewait + 1;
        if (Prewait >= 100000)
        {
            return;                           // time out error
        }
    }
    // request power active mode LDO VCORE1 to support the 48 MHz frequency
    PCM->CTL0 = (PCM->CTL0 & ~0xFFFF000F) | // clear PCMKEY bit field and AMR bit field
//  PCMCTL0 = (PCMCTL0&~0xFFFF000F) |     // clear PCMKEY bit field and AMR bit field
            0x695A0000 |     // write the proper PCM key to unlock write access
            0x00000001;                 // request power active mode LDO VCORE1
    // check if the transition is invalid (see Figure 7-3 on p344 of datasheet)
    if (PCM->IFG & 0x00000004)
    {
        IFlags = PCM->IFG; // bit 2 set on active mode transition invalid; bits 1-0 are for LPM-related errors; bit 6 is for DC-DC-related error
        PCM->CLRIFG = 0x00000004;           // clear the transition invalid flag
        // to do: look at CPM bit field in PCMCTL0, figure out what mode you're in, and step through the chart to transition to the mode you want
        // or be lazy and do nothing; this should work out of reset at least, but it WILL NOT work if Clock_Int32kHz() or Clock_InitLowPower() has been called
        return;
    }
    // wait for the CPM (Current Power Mode) bit field to reflect a change to active mode LDO VCORE1
    while ((PCM->CTL0 & 0x00003F00) != 0x00000100)
    {
        CPMwait = CPMwait + 1;
        if (CPMwait >= 500000)
        {
            return;                           // time out error
        }
    }
    // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
    while (PCM->CTL1 & 0x00000100)
    {
        Postwait = Postwait + 1;
        if (Postwait >= 100000)
        {
            return;                           // time out error
        }
    }
    // initialize PJ.3 and PJ.2 and make them HFXT (PJ.3 built-in 48 MHz crystal out; PJ.2 built-in 48 MHz crystal in)
    PJ->SEL0 |= 0x0C;
    PJ->SEL1 &= ~0x0C;   // configure built-in 48 MHz crystal for HFXT operation
    CS->KEY = 0x695A;                    // unlock CS module for register access
    CS->CTL2 = (CS->CTL2 & ~0x00700000) |   // clear HFXTFREQ bit field
            0x00600000 |               // configure for 48 MHz external crystal
            0x00010000 | // HFXT oscillator drive selection for crystals >4 MHz
            0x01000000;                  // enable HFXT
    CS->CTL2 &= ~0x02000000;            // disable high-frequency crystal bypass
    // wait for the HFXT clock to stabilize
    while (CS->IFG & 0x00000002)
    {
        CS->CLRIFG = 0x00000002;     // clear the HFXT oscillator interrupt flag
        Crystalstable = Crystalstable + 1;
        if (Crystalstable > 100000)
        {
            return;                           // time out error
        }
    }
    // configure for 2 wait states (minimum for 48 MHz operation) for flash Bank 0
    FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL & ~0x0000F000)
            | FLCTL_BANK0_RDCTL_WAIT_2;
    // configure for 2 wait states (minimum for 48 MHz operation) for flash Bank 1
    FLCTL->BANK1_RDCTL = (FLCTL->BANK1_RDCTL & ~0x0000F000)
            | FLCTL_BANK1_RDCTL_WAIT_2;
    CS->CTL1 = 0x20000000 |               // configure for SMCLK divider /4
            0x00100000 |                 // configure for HSMCLK divider /2
            0x00000200 |             // configure for ACLK sourced from REFOCLK
            0x00000050 | // configure for SMCLK and HSMCLK sourced from HFXTCLK
            0x00000005;               // configure for MCLK sourced from HFXTCLK
    CS->KEY = 0;                        // lock CS module from unintended access
    ClockFrequency = 48000000;
//  SubsystemFrequency = 12000000;
}


/*
 * Initializes launchpad LEDs and buttons
 */
void LaunchPad_Init(void)
{
    P1->SEL0 &= ~0x13;
    P1->SEL1 &= ~0x13;    // 1) configure P1.4 and P1.1 as GPIO
    P1->DIR &= ~0x12;     // 2) make P1.4 and P1.1 in
    P1->DIR |= 0x01;      //    make P1.0 out
    P1->REN |= 0x12;      // 3) enable pull resistors on P1.4 and P1.1
    P1->OUT |= 0x12;      //    P1.4 and P1.1 are pull-up
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;    // 1) configure P2.2-P2.0 as GPIO
    P2->DIR |= 0x07;      // 2) make P2.2-P2.0 out
    P2->DS |= 0x07;       // 3) activate increased drive strength
    P2->OUT &= ~0x07;     //    all LEDs off
}

/*
 * Initialized the motors
 */
void Motor_Init(void)
{
    P1->SEL0 &= ~0xC0; //1.6, 1.7 as output
    P1->SEL1 &= ~0xC0;
    P1->DIR |= 0xC0;
    //P1->DS |= 0xC0;

    P3->SEL0 &= ~0xC0; //3.6, 3.7
    P3->SEL1 &= ~0xC0;
    P3->DIR |= 0xC0;
    //P3->DS |= 0xC0;

    P1->OUT &= ~0xC0;
    P3->OUT &= ~0xC0;   // low current sleep mode

}

/*
 * Initializes TimerA0 to generate PWM and sets up output pins
 */
void PWM_init(uint16_t period, uint16_t duty1, uint16_t duty2)
{
    P2->DIR |= BIT6 | BIT7;
    P2->SEL0 |= BIT6 | BIT7;
    P2->SEL1 &= BIT6 | BIT7;

    TIMER_A0->CCTL[0] = 0x0080;
    TIMER_A0->CCR[0] = period;

    TIMER_A0->EX0 = 0x0000;

    TIMER_A0->CCTL[3] = 0x0040;
    TIMER_A0->CCR[3] = duty1;

    TIMER_A0->CCTL[4] = 0x0040;
    TIMER_A0->CCR[4] = duty2;

    TIMER_A0->CTL = 0x02F0;

    PERIOD = period;
}

/**
 * Initializes TimerA2 to do PWM for the status LEDs
 */
void status_LED_init(uint16_t period, uint16_t duty1, uint16_t duty2,
                     uint16_t duty3)
{
    P5->DIR |= BIT6 | BIT7;
    P6->DIR |= BIT6;

    P5->SEL0 |= BIT6 | BIT7;
    P5->SEL1 &= ~BIT6 | ~BIT7;

    P6->SEL0 |= BIT6;
    P6->SEL1 &= ~BIT6;

    TIMER_A2->CCTL[0] = 0x0080;
    TIMER_A2->CCR[0] = period;

    TIMER_A2->CCTL[1] = 0x0040;
    TIMER_A2->CCR[1] = duty1;

    TIMER_A2->CCTL[2] = 0x0040;
    TIMER_A2->CCR[2] = duty2;

    TIMER_A2->CCTL[3] = 0x0040;
    TIMER_A2->CCR[3] = duty3;

    TIMER_A2->CTL = 0x0130;
    TIMER_A2->EX0 = 0x0000;
}

/*
 * sets the duty cycle of the status LEDs
 */
void set_status_leds_duty(uint16_t duty1, uint16_t duty2, uint16_t duty3)
{
    TIMER_A2->CCR[1] = duty1;
    TIMER_A2->CCR[2] = duty2;
    TIMER_A2->CCR[3] = duty3;
}

/*
 * Sets the power of the right motors
 * power is between -100 and 100
 */
void set_right_motor_power(int8_t power_level)
{
    set_motor_power(power_level, Right);
}

/*
 * sets the power of the left motor
 * power is between -100 and 100
 */
void set_left_motor_power(int8_t power_level)
{
    set_motor_power(power_level, Left);
}

/*
 * sets the power of the specified motors
 * power is between -100 (reversed) and 100 (forward)
 */
void set_motor_power(int8_t power_level, enum motor m)
{
    uint16_t duty;

    //if the motor is not stopped
    if (power_level != 0)
    {
        //inverse the motor if the speed is negative
        if (power_level < 0)
        {
            P1->OUT |= m;
            power_level = power_level * -1;
        }
        else
        {
            P1->OUT &= ~m;
        }

        //clip the power level to 100
        if (power_level > 100)
        {
            power_level = 100;
        }

        //Determine the duty cycle based off the power level and the period
        duty = ((uint16_t) power_level * PERIOD) / 100;
        P3->OUT |= m;
    }
    else
    {
        //set duty to 0 and turn off the motors
        duty = 0;
        P3->OUT &= ~m;
    }

    //Set the appropriate motor's duty
    if (m == Right)
    {
        set_right_motor_duty(duty);
    }
    else
    {
        set_left_motor_duty(duty);
    }
}

/**
 * set's the left motor's duty
 */
void set_left_motor_duty(uint16_t duty)
{
    TIMER_A0->CCR[4] = duty;
}

/**
 * set's the right motor's duty
 */
void set_right_motor_duty(uint16_t duty)
{
    TIMER_A0->CCR[3] = duty;
}

/**
 * Reads the inputs from launchpad's buttons
 */
uint8_t LaunchPad_Input(void)
{
    return ((((~(P1->IN)) & 0x10) >> 3) | (((~(P1->IN)) & 0x02) >> 1)); // read P1.4,P1.1 inputs
}

/**
 * Sets the output of the RGB led
 * BIT0 Red
 * BIT1 Green
 * BIT2 Blue
 */
void LaunchPad_Output(uint8_t data)
{  // write three outputs bits of P2
    P2->OUT = (P2->OUT & 0xF8) | data;
}

/**
 * Waits for a button to be pressed
 */
void Pause(void)
{
    while (LaunchPad_Input() == 0);  // wait for touch
    while (LaunchPad_Input()); // wait for release
}

/**
 * Initializes P4 for the bump switches
 */
void BumpInt_Init(void (*task)(uint8_t))
{
    //Setup P4 pins
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;
    P4->DIR &= ~0xED;
    P4->REN |= 0xED;
    P4->OUT |= 0xED;
    bump_event = task;

    //Setup P4 Interrupts
    P4->IES &= ~0xED;
    P4->IFG &= ~0xED;
    P4->IE |= 0xED;

    NVIC->IP[9] = (NVIC->IP[9] & 0xFF00FFFF) | 0x00040000;
    NVIC->ISER[1] = 0x00000040;
}

/**
 * Reads the bump switch on the front of the robot
 */
uint8_t Bump_Read(void)
{
    bump = P4->IN;
    bump0 = ~(P4->IN) & 0x01;        //0
    bump1 = (~(P4->IN) & 0x04) >> 1; //1
    bump2 = (~(P4->IN) & 0x08) >> 1; ///2
    bump3 = (~(P4->IN) & 0x20) >> 2; //3
    bump4 = (~(P4->IN) & 0x40) >> 2; //4
    bump5 = (~(P4->IN) & 0x80) >> 2; //5

    return (bump0 | bump1 | bump2 | bump3 | bump4 | bump5); //return the value of the bump switches
}

/**
 * Takes corrective action to follow the line, based off the input distance
 */
void follow_line(int16_t distance_from_line)
{
    //Base power of the motors
    unsigned int base_power = BASE_POWER;
    //Change in the base power
    int power;

    //Power of each motor
    int right_motor_power;
    int left_motor_power;

    int forward;

    if (distance_from_line < 0) {
        distance_from_line = distance_from_line * -1;
        forward = 0;
    }
    else {
        forward = 1;
    }

    //Find how much how power is needed based of the distance
    power = ((((uint16_t) distance_from_line << 2) / (uint16_t) 332)
            * (base_power)) >> 2;

    //Set the power based off which way needs to be corrected
    if (!forward)
    {
        right_motor_power = base_power - power;
        left_motor_power = base_power + power;
    }
    else
    {
        right_motor_power = base_power + power;
        left_motor_power = base_power - power;
    }

    //Set motor power
    set_right_motor_power(right_motor_power);
    set_left_motor_power(left_motor_power);
}

/**
 * Gets the distance away from the line
 */
int16_t get_distance_from_line(uint8_t line_value)
{
    //Array holding the distance from each sensor from the mid point
    int16_t distances[] = { -332, -237, -142, -47, 47, 237, 332 };
    int16_t sum = 0;
    int16_t divisor = 0;
    uint8_t weight;
    uint8_t i;

    //Find the average distance
    for (i = 0; i < 8; i++)
    {
        weight = ((line_value & (BIT0 << i)) >> i);
        sum += distances[i] * weight;
        divisor += weight;
    }

    return sum / divisor;
}

/**
 * Initialized the line follower
 */
void line_follower_init() {
    //Setup line follower port
    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF;
    P7->DIR &= ~0xFF;
    P7->DS |= 0xFF;
    P7->REN &= ~0xFF;
    P7->OUT |= 0xFF;

    //Setup IR led driver pin
    P5->SEL0 &= ~BIT3;
    P5->SEL1 &= ~BIT3;
    P5->DIR |= BIT3;
    P5->OUT &= ~BIT3;
    P5->DS |= BIT3;
}

/**
 * Begins the process of reading the line sensor
 */
void line_sensor_begin_read()
{
    //Turn on IR LEDS
    P5->OUT |= BIT3;

    //Set P7 High
    P7->DIR = 0xFF;
    P7->OUT = 0xFF;

    //Wait 10us
    Clock_Delay1us(10);

    //Set P7 as input
    P7->DIR = 0x00;
}

/**
 * Finishes reading the line sensor
 */
uint8_t line_sensor_end_read()
{
    //Store the result
    uint8_t result = P7->IN;

    //Turn off IR LED
    P5->OUT &= ~BIT3;
    return result;
}

/**
 * Handles reading the line sensor
 */
uint8_t count = 1;
volatile uint8_t line_sensor_raw = 0;
void SysTick_Handler(void)
{
    //Begin reading the line sensor
    if (count == 1)
    {
        line_sensor_begin_read();
    }
    //Finish reading the line sensor
    else if (count == 20)
    {
        line_sensor_raw = line_sensor_end_read();
        //reset count
        count = 0;
    }

    count++;
}

/**
 * Handles the bump event
 */
void PORT4_IRQHandler(void)
{
    P4->IFG &= ~0xEF;
    //Ensure bump switches have been fired
    uint8_t status = Bump_Read();
    if (status)
    {
        //call bump event
        (*bump_event)(status);
    }
}

/*
 * Moves the state machine to the collision state
 */
void HandleCollision(uint8_t bumpSensor)
{
    state = COLLISION;
}

#define HALF_HERTZ_PERIOD 16000
#define HALF_DUTY 8000

int main(void)
{
    //Distance away from the line
    int16_t distance_from_line;

    //Used to track turn status
    uint8_t line_cross = 0;
    uint8_t last_status = 0;

    //Last turn taken
    enum STATE last_turn = RIGHT_TURN;

    //Initialize Everything!!
    Clock_Init48MHz();
    line_follower_init();
    SysTick_Init();
    Motor_Init();
    BumpInt_Init(*HandleCollision);
    LaunchPad_Init();
    PWM_init(8000, 0, 0);
    status_LED_init(HALF_HERTZ_PERIOD, HALF_DUTY, HALF_DUTY, HALF_DUTY);
    EnableInterrupts();

    //Wait for the user to begin
    Pause();

    //Run for ever
    while (1)
    {
        //Get distance from the line
        distance_from_line = get_distance_from_line(line_sensor_raw);

        //If we found the treasure,
        if (((line_sensor_raw & 0xBD) == 0x99))
        {
            state = DONE;
        }

        //Straight State
        if (state == STRAIGHT)
        {
            //turn off the status LEDs
            set_status_leds_duty(0, 0, 0);

            //if we find a turn, go to the turn state
            if ((line_sensor_raw & 0x03) || (line_sensor_raw & 0xC0))
            {
                state = TURN;
            }
            //If there is any line data, try and follow it
            else if (line_sensor_raw)
            {
                follow_line(distance_from_line);
            }
            //if no line data is found, turn around
            else
            {
                if (distance_from_line < 0)
                {
                    set_right_motor_power(15);
                    set_left_motor_power(-15);
                }
                else
                {
                    set_right_motor_power(-15);
                    set_left_motor_power(15);
                }
                state = TURN_AROUND;
                set_status_leds_duty(0, 0, HALF_DUTY);
            }
        }
        //if we are in a turn state
        else if (state == TURN)
        {
            //If we are at a t intersection, pick the last direction we took
            if ((line_sensor_raw & 0xE0) && (line_sensor_raw & 0x07))
            {
                state = last_turn;

                //move forward slightly
                while (((line_sensor_raw & 0x03) == 0x3)
                        && ((line_sensor_raw & 0xC0) == 0xC0))
                {
                }
            }
            //if we have a left turn
            else if (line_sensor_raw & 0xC0)
            {
                state = LEFT_TURN;
                //move forward slightly
                while ((line_sensor_raw & 0xC0) == 0xC0)
                {
                }
            }
            //if we have a right turn
            else
            {
                state = RIGHT_TURN;
                while (((line_sensor_raw & 0x03) == 0x03))
                {
                }
            }
            //update last turn with state
            last_turn = state;

        }
        //if we are turning right
        else if (state == RIGHT_TURN)
        {
            //Start spinning
            if (line_cross == 0)
            {
                set_status_leds_duty(0, HALF_DUTY, 0);
                set_right_motor_power(-15);
                set_left_motor_power(30);

            }

            //if we have not crossed the line
            if (line_cross < 2)
            {
                line_cross += ((line_sensor_raw & BIT2 ) >> 2) ^ last_status;
                last_status = (line_sensor_raw & BIT2 ) >> 2;
            }
            else
            {
                //Reset and drive straight
                line_cross = 0;
                last_status = 0;
                set_right_motor_power(35);
                set_left_motor_power(35);
                state = STRAIGHT;
            }
        }
        else if (state == LEFT_TURN)
        {
            //Start spinning
            if (line_cross == 0)
            {
                set_status_leds_duty(HALF_DUTY, 0, 0);
                set_right_motor_power(30);
                set_left_motor_power(-15);
            }
            //if we have not crossed the line
            if (line_cross < 2)
            {
                line_cross += ((line_sensor_raw & BIT4 ) >> 4) ^ last_status;
                last_status = (line_sensor_raw & BIT4 ) >> 4;
            }
            else
            {
                //Reset and drive straight
                line_cross = 0;
                last_status = 0;
                set_right_motor_power(35);
                set_left_motor_power(35);
                state = STRAIGHT;
            }
        }
        //If we got a collision
        else if (state == COLLISION)
        {
            //while are bump switches are still pressed, back up
            if (Bump_Read())
            {
                set_right_motor_power(-35);
                set_left_motor_power(-35);
            }
            //When we have cleared the collision, turn around
            else
            {
                //delay some time to make sure we are clear
                Clock_Delay1us(500);

                //Find best way to spin
                if (distance_from_line < 0)
                {
                    set_right_motor_power(15);
                    set_left_motor_power(-15);
                }
                else
                {
                    set_right_motor_power(-15);
                    set_left_motor_power(15);
                }

                //go to turn around state
                state = TURN_AROUND;

                //turn on status leds
                set_status_leds_duty(0, 0, HALF_DUTY);
            }
        }
        //if we need to turn around
        else if (state == TURN_AROUND)
        {
            //If we have not crossed the line twice
            if (line_cross < 2)
            {
                line_cross += ((line_sensor_raw & BIT3 ) >> 3) ^ last_status;
                last_status = (line_sensor_raw & BIT3 ) >> 3;
            }
            //When we have cross the line, reset the state and go to straight
            else
            {
                line_cross = 0;
                last_status = 0;
                set_right_motor_power(35);
                set_left_motor_power(35);
                state = STRAIGHT;
            }
        }
        //if we found the treasure
        else if (state == DONE)
        {
            //set the status LEDs
            status_LED_init(HALF_HERTZ_PERIOD / 2, HALF_DUTY / 2, HALF_DUTY / 2,
                            HALF_DUTY / 2);
            LaunchPad_Output(3);

            //stop motors
            set_right_motor_power(0);
            set_left_motor_power(0);

            while (1); //bye forever
        }
    }
}
