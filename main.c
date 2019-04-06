/*
 * TODO
 * + Finish line follower code
 * + Add collision handler
 * + Add path finding logic (need to build the maze)
 * + Test and optimize
 */

#include "msp.h"
#include "CortexM.h"

//Possible Motor Values
enum motor{Right = BIT6, Left = BIT7};

int16_t right_motor_speed;
int16_t left_motor_speed;

//Line Data
int16_t line_data;

//TimerA0 Period
uint16_t PERIOD;

//Clock Globals
uint32_t ClockFrequency = 3000000; // cycles/second

int32_t Prewait = 0;                   // loops between BSP_Clock_InitFastest() called and PCM idle (expect 0)
uint32_t CPMwait = 0;                   // loops between Power Active Mode Request and Current Power Mode matching requested mode (expect small)
uint32_t Postwait = 0;                  // loops between Current Power Mode matching requested mode and PCM module idle (expect about 0)
uint32_t IFlags = 0;                    // non-zero if transition is invalid
uint32_t Crystalstable = 0;             // loops before the crystal stabilizes (expect small)

volatile uint8_t bump;
uint8_t bump0, bump1, bump2, bump3, bump4, bump5;
uint8_t Bump_Read();
void(*bump_event)(uint8_t);
void(*line_follower_event)(uint8_t);
void set_left_motor_duty(uint16_t);
void set_right_motor_duty(uint16_t);
void set_motor_power(int8_t power_level, enum motor m);
uint8_t line_follower_read(void);

void delay(unsigned long ulCount){
  __asm (  "pdloop:  subs    r0, #1\n"
      "    bne    pdloop\n");
}


// ------------Clock_Delay1ms------------
// Simple delay function which delays about n milliseconds.
// Inputs: n, number of msec to wait
// Outputs: none
void Clock_Delay1ms(uint32_t n) {
  while(n){
    delay(ClockFrequency/9162);   // 1 msec, tuned at 48 MHz
    n--;
  }
}


/****************************************
 *  SysTick Timer Init
 ***************************************/
void SysTick_Init(void){
  SysTick->CTRL = 0;
  SysTick->LOAD = 0x00FFFFFF;           // maximum reload value
  SysTick->VAL = 0;
  SysTick->CTRL = 0x00000005;           // enable SysTick with no interrupts

}

void SysTick_Wait(uint32_t delay){
  SysTick->LOAD = (delay - 1);// count down to zero
  SysTick->VAL = 0;          // any write to CVR clears it and COUNTFLAG in CSR
  while(( SysTick->CTRL&0x00010000) == 0){};

}
// Time delay using busy wait.
// assumes 48 MHz bus clock
void SysTick_Wait10ms(uint32_t delay){
  uint32_t i;
  for(i=0; i<delay; i++){
    SysTick_Wait(480000);  // wait 10ms (assumes 48 MHz clock)
  }
}

void SysTick_Delay1us(uint32_t delay){
  uint32_t i;
  for(i=0; i<delay/2; i++){
    SysTick_Wait(2);  // wait 1ms (assumes 48 MHz clock)
  }
}

void Clock_Delay1us(uint32_t n) {
    n = (382*n)/100;

    while(n) {
        n--;
    }
}

// delay function
// which delays about 6*ulCount cycles
// ulCount=8000 => 1ms = (8000 loops)*(6 cycles/loop)*(20.83 ns/cycle)
  //Code Composer Studio Code
void Clock_Init32KHz(void){
  // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
  while(PCM->CTL1&0x00000100){
//  while(PCMCTL1&0x00000100){
    Prewait = Prewait + 1;
    if(Prewait >= 100000){
      return;                           // time out error
    }
  }
  // request power active mode LDO VCORE1 to support the 48 MHz frequency
  PCM->CTL0 = (PCM->CTL0&~0xFFFF000F) |     // clear PCMKEY bit field and AMR bit field
//  PCMCTL0 = (PCMCTL0&~0xFFFF000F) |     // clear PCMKEY bit field and AMR bit field
            0x695A0000 |                // write the proper PCM key to unlock write access
            0x00000001;                 // request power active mode LDO VCORE1
  // check if the transition is invalid (see Figure 7-3 on p344 of datasheet)
  if(PCM->IFG&0x00000004){
    IFlags = PCM->IFG;                    // bit 2 set on active mode transition invalid; bits 1-0 are for LPM-related errors; bit 6 is for DC-DC-related error
    PCM->CLRIFG = 0x00000004;             // clear the transition invalid flag
    // to do: look at CPM bit field in PCMCTL0, figure out what mode you're in, and step through the chart to transition to the mode you want
    // or be lazy and do nothing; this should work out of reset at least, but it WILL NOT work if Clock_Int32kHz() or Clock_InitLowPower() has been called
    return;
  }
  // wait for the CPM (Current Power Mode) bit field to reflect a change to active mode LDO VCORE1
  while((PCM->CTL0&0x00003F00) != 0x00000100){
    CPMwait = CPMwait + 1;
    if(CPMwait >= 500000){
      return;                           // time out error
    }
  }
  // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
  while(PCM->CTL1&0x00000100){
    Postwait = Postwait + 1;
    if(Postwait >= 100000){
      return;                           // time out error
    }
  }
  // initialize PJ.3 and PJ.2 and make them HFXT (PJ.3 built-in 48 MHz crystal out; PJ.2 built-in 48 MHz crystal in)
  PJ->SEL0 |= 0x03;        // 0000 0011
  PJ->SEL1 &= ~0x03;   // 1111 1100  configure built-in 32 kHz crystal for LFXT

  CS->KEY = 0x695A;            // unlock CS module for register access
  CS->CTL2 = (CS->CTL2 &~0x00000003)|0x00000003|0x00000100;
  CS->CTL2 &= ~0x00000200;     //disable low-frequency crystal bypass

  // wait for LXFT clock to stabilize
  while(CS->IFG&0x00000001){
     CS->CLRIFG = 0x00000001;       // clear the LFXT interrupt flag
     Crystalstable=Crystalstable+1;
    if(Crystalstable > 100000)
      return ;            // time out error
  }
  CS->CTL1 = 0x10000200;       // SMCLK/2 HSMCLK MCLK from LXFT
  CS->KEY = 0;         // lock CS module                         // lock CS module from unintended access
  ClockFrequency = 48000000;
//  SubsystemFrequency = 12000000;
}


void Clock_Init48MHz(void){
  // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
  while(PCM->CTL1&0x00000100){
//  while(PCMCTL1&0x00000100){
    Prewait = Prewait + 1;
    if(Prewait >= 100000){
      return;                           // time out error
    }
  }
  // request power active mode LDO VCORE1 to support the 48 MHz frequency
  PCM->CTL0 = (PCM->CTL0&~0xFFFF000F) |     // clear PCMKEY bit field and AMR bit field
//  PCMCTL0 = (PCMCTL0&~0xFFFF000F) |     // clear PCMKEY bit field and AMR bit field
            0x695A0000 |                // write the proper PCM key to unlock write access
            0x00000001;                 // request power active mode LDO VCORE1
  // check if the transition is invalid (see Figure 7-3 on p344 of datasheet)
  if(PCM->IFG&0x00000004){
    IFlags = PCM->IFG;                    // bit 2 set on active mode transition invalid; bits 1-0 are for LPM-related errors; bit 6 is for DC-DC-related error
    PCM->CLRIFG = 0x00000004;             // clear the transition invalid flag
    // to do: look at CPM bit field in PCMCTL0, figure out what mode you're in, and step through the chart to transition to the mode you want
    // or be lazy and do nothing; this should work out of reset at least, but it WILL NOT work if Clock_Int32kHz() or Clock_InitLowPower() has been called
    return;
  }
  // wait for the CPM (Current Power Mode) bit field to reflect a change to active mode LDO VCORE1
  while((PCM->CTL0&0x00003F00) != 0x00000100){
    CPMwait = CPMwait + 1;
    if(CPMwait >= 500000){
      return;                           // time out error
    }
  }
  // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
  while(PCM->CTL1&0x00000100){
    Postwait = Postwait + 1;
    if(Postwait >= 100000){
      return;                           // time out error
    }
  }
  // initialize PJ.3 and PJ.2 and make them HFXT (PJ.3 built-in 48 MHz crystal out; PJ.2 built-in 48 MHz crystal in)
  PJ->SEL0 |= 0x0C;
  PJ->SEL1 &= ~0x0C;                    // configure built-in 48 MHz crystal for HFXT operation
  CS->KEY = 0x695A;                     // unlock CS module for register access
  CS->CTL2 = (CS->CTL2&~0x00700000) |   // clear HFXTFREQ bit field
           0x00600000 |                 // configure for 48 MHz external crystal
           0x00010000 |                 // HFXT oscillator drive selection for crystals >4 MHz
           0x01000000;                  // enable HFXT
  CS->CTL2 &= ~0x02000000;              // disable high-frequency crystal bypass
  // wait for the HFXT clock to stabilize
  while(CS->IFG&0x00000002){
    CS->CLRIFG = 0x00000002;              // clear the HFXT oscillator interrupt flag
    Crystalstable = Crystalstable + 1;
    if(Crystalstable > 100000){
      return;                           // time out error
    }
  }
  // configure for 2 wait states (minimum for 48 MHz operation) for flash Bank 0
  FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL&~0x0000F000)|FLCTL_BANK0_RDCTL_WAIT_2;
  // configure for 2 wait states (minimum for 48 MHz operation) for flash Bank 1
  FLCTL->BANK1_RDCTL = (FLCTL->BANK1_RDCTL&~0x0000F000)|FLCTL_BANK1_RDCTL_WAIT_2;
  CS->CTL1 = 0x20000000 |               // configure for SMCLK divider /4
           0x00100000 |                 // configure for HSMCLK divider /2
           0x00000200 |                 // configure for ACLK sourced from REFOCLK
           0x00000050 |                 // configure for SMCLK and HSMCLK sourced from HFXTCLK
           0x00000005;                  // configure for MCLK sourced from HFXTCLK
  CS->KEY = 0;                          // lock CS module from unintended access
  ClockFrequency = 48000000;
//  SubsystemFrequency = 12000000;
}


/*************************************
 *  Launchpad init
 ************************************/
void LaunchPad_Init(void){
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


void Motor_Init(void){
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

void PWM_init(uint16_t period, uint16_t duty1, uint16_t duty2) {
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

void set_right_motor_power(int8_t power_level) {
    set_motor_power(power_level, Right);
}

void set_left_motor_power(int8_t power_level) {
    set_motor_power(power_level, Left);
}

void set_motor_power(int8_t power_level, enum motor m) {
    uint16_t duty;


    if (power_level != 0) {
        if (power_level < 0) {
            P1->OUT |= m;
            power_level = power_level*-1;
        }
        else {
            P1-> OUT & ~m;
        }

        if (power_level > 100) {
            power_level = 100;
        }

        duty = ((uint16_t)power_level * PERIOD)/100;
        P3->OUT |= m;
    }
    else {
        duty = 0;
        P3->OUT &= ~m;
    }

    if (m == Right) {
        set_right_motor_duty(duty);
    }
    else {
        set_left_motor_duty(duty);
    }
}


void set_left_motor_duty(uint16_t duty) {
    TIMER_A0->CCR[4] = duty;
}

void set_right_motor_duty(uint16_t duty) {
    TIMER_A0->CCR[3] = duty;
}



//------------LaunchPad_Input------------
// Input from Switches
// Input: none
// Output: 0x00 none
//         0x01 Button1
//         0x02 Button2
//         0x03 both Button1 and Button2
uint8_t LaunchPad_Input(void){
  return ((((~(P1->IN))&0x10)>>3)|(((~(P1->IN))&0x02)>>1));   // read P1.4,P1.1 inputs
}

//------------LaunchPad_Output------------
// Output to LaunchPad LEDs
// Input: 0 off, bit0=red,bit1=green,bit2=blue
// Output: none
void LaunchPad_Output(uint8_t data){  // write three outputs bits of P2
  P2->OUT = (P2->OUT&0xF8)|data;
}


// Driver test
void Pause(void){
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}



// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
// Interrupt on falling edge (on touch)
void BumpInt_Init(void(*task)(uint8_t)){
    // write this as part of Lab 7
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;
    P4->DIR &= ~0xED;
    P4->REN |= 0xED;
    P4->OUT |= 0xED;
    bump_event = task;

    P4->IES |= 0xED;
    P4->IFG &= ~0xED;
    P4->IE |= 0xED;

    NVIC->IP[9]=(NVIC->IP[9]&0xFF00FFFF) | 0x00040000;
    NVIC->ISER[1] = 0x00000040;
}

// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump5
// bit 4 Bump4
// bit 3 Bump3
// bit 2 Bump2
// bit 1 Bump1
// bit 0 Bump0

uint8_t Bump_Read(void){
    bump = P4->IN;
    bump0 = ~(P4->IN) & 0x01;        //0
    bump1 = (~(P4->IN) & 0x04) >> 1; //1
    bump2 = (~(P4->IN) & 0x08) >> 1; ///2
    bump3 = (~(P4->IN) & 0x20) >> 2; //3
    bump4 = (~(P4->IN) & 0x40) >> 2; //4
    bump5 = (~(P4->IN) & 0x80) >> 2; //5

    return (bump0 | bump1 | bump2 | bump3 | bump4 | bump5); // replace this line
}

void follow_line(int16_t distance_from_line){
    unsigned int base_power = 50;
    int right_motor_power;
    int left_motor_power;

    if (distance_from_line < 0) {
        distance_from_line *= -1;
        right_motor_power = base_power;
        left_motor_power = ((((((uint16_t)distance_from_line << 2)/(uint16_t)332) * (base_power)) + (base_power << 2)) >> 2);
    }
    else {
        right_motor_power = ((((((uint16_t)distance_from_line << 2)/(uint16_t)332) * (base_power)) + (base_power << 2)) >> 2);
        left_motor_power = base_power;
    }
    left_motor_speed = left_motor_power;
    right_motor_speed = right_motor_power;
    set_right_motor_power(right_motor_power);
    set_left_motor_power(left_motor_power);
}


int16_t get_distance_from_line(uint8_t line_value) {
    int16_t distances[] = {-332, -237, -142, -47, 47, 237, 332};
    int16_t sum = 0;
    int16_t divisor = 0;
    uint8_t weight;
    uint8_t i;

    for (i=0; i < 8; i++) {
        weight = ((line_value & (BIT0 << i) ) >> i);
        sum += distances[i]*weight;
        divisor += weight;
    }

    return sum/divisor;
}

uint8_t line_follower_read(void) {
    uint8_t result;
    //Turn on IR LEDS
    P5->OUT |= BIT3;

    //Set P7 High
    P7->DIR = 0xFF;
    P7->OUT = 0xFF;

    //Wait 10us
    Clock_Delay1us(10);
    //SysTick_Delay1us(15);

    //Make P7 input
    P7->DIR = 0x00;

    //Allow RC circuit to decay
    //SysTick_Delay1us(1400);
    Clock_Delay1us(2000);

    //read data
    result = P7->IN;

    //Turn off IR LED
    P5->OUT &= ~BIT3;

    //wait for everything to turn off
    //SysTick_Delay1us(15);
    Clock_Delay1us(10);
    return result;
}


// we do not care about critical section/race conditions
// triggered on touch, falling edge
void PORT4_IRQHandler(void){
    P4->IFG &= ~0xEF;
    uint8_t status = Bump_Read();
    if (status) {
        (*bump_event)(status);
    }
}
/***************************
 * main.c
 ***************************/
uint8_t CollisionData, CollisionFlag;  // mailbox
void HandleCollision(uint8_t bumpSensor){
   CollisionData = bumpSensor;
   CollisionFlag = 1;
}

int main(void){  // test of interrupt-driven bump interface

  uint8_t line_follower_data;
  int16_t distance_from_line;
  Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clock
  CollisionFlag = 0;

  SysTick_Init();
  Motor_Init();
  LaunchPad_Init();
  PWM_init(8000, 0, 0);
  EnableInterrupts();

  P7->SEL0 &= ~0xFF;
  P7->SEL1 &= ~0xFF;
  P7->DIR &= ~0xFF;
  P7->DS |= 0xFF;
  P7->REN &= ~0xFF;
  P7->OUT |= 0xFF;

  P5->SEL0 &= ~BIT3;
  P5->SEL1 &= ~BIT3;
  P5->DIR |= BIT3;
  P5->OUT &= ~BIT3;
  P5->DS |= BIT3;

  Pause();
  //set_right_motor_power(50);
  //set_left_motor_power(50);

  while(1){
      line_follower_data = line_follower_read();

      if (line_follower_data) {
          distance_from_line = get_distance_from_line(line_follower_data);
          follow_line(distance_from_line);
      }
      else {
          set_right_motor_power(0);
          set_left_motor_power(0);
      }
  }
}
