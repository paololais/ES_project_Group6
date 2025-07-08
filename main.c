/*
 * File:   main.c
 * Author: paolo
 *
 * Created on May 19, 2025, 10:38 AM
 */


#include "xc.h"
#include "scheduler.h"
#include "parser.h"
#include "pwm.h"
#include "timer.h"
#include "uart.h"
#include "adc.h"

int PWM_state = 0; // 0=off, 1=on

// Interrupt INT1 (button RE8)
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt(){
  IFS1bits.INT1IF = 0; 
  IEC0bits.T3IE = 1; // 
  tmr_setup_period(TIMER3, 10); 
}

// Interrupt Timer 3 for debouncing
void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt(){
  IFS0bits.T3IF = 0;
  IEC0bits.T3IE = 0; 
  T3CONbits.TON = 0;
  IEC1bits.INT1IE = 1;

  if (PORTEbits.RE8 == 1) {
      PWM_state = !PWM_state;
  }
}

int main(void) {       
    
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
     
    // button RE8 configuration
    TRISEbits.TRISE8 = 1; 
    RPINR0bits.INT1R = 0x58; 
    INTCON2bits.GIE = 1; 
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 1;
    
    pwm_init();
    
    while(1){
        if(PWM_state == 1){
            OC1R = 0;
            OC2R = 2000;
            OC3R = 0;
            OC4R = 7200;
        } else {
            pwm_stop();
        }
    }
    
    return 0;
}
