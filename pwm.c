/*
 * File:   pwm.c
 * Author: paolo
 *
 * Created on 13 maggio 2025, 10.58
 */


#include "pwm.h"

void pwm_init(){
    OC1CON1 = 0; // good practice to clear off the control bits initially
    OC1CON2 = 0; // good practice to clear off the control bits initially
    
    // setting as output
    TRISDbits.TRISD1 = 0; // set as output
    TRISDbits.TRISD2 = 0;
    TRISDbits.TRISD3 = 0; // set as output
    TRISDbits.TRISD4 = 0; // set as output to move forward  
    
    
    // pin remap for pwm (H-bridge)
    RPOR0bits.RP65R = 0b010000; // RD1 corresponds to OC1 the remappable pin RD1 = RP65R
    RPOR1bits.RP66R = 0b010001; // RD2 corresponds to OC2 the remappable pin RD1 = RP66R
    RPOR1bits.RP67R = 0b010010; // RD3 corresponds to OC3 the remappable pin RD1 = RP67R
    RPOR2bits.RP68R = 0b010011; // RD4 corresponds to OC1 the remappable pin RD1 = RP68R
    
    // configure the output compare 
    OC1CON1bits.OCM = 0b000; // disable before to cofigure
    OC2CON1bits.OCM = 0b000; // disable before to cofigure
    OC3CON1bits.OCM = 0b000; // disable before to cofigure
    OC4CON1bits.OCM = 0b000; // disable before to cofigure
    
    // select for clock for the output compare
    OC1CON1bits.OCTSEL = 0b111; // select the input clock  source for the OC1module 
    OC2CON1bits.OCTSEL = 0b111; // select the input clock  source for the OC1module 
    OC3CON1bits.OCTSEL = 0b111; // select the input clock  source for the OC1module 
    OC4CON1bits.OCTSEL = 0b111; // select the input clock  source for the OC1module 
     
    // as peripheral clock that is the same to microcontroller
    OC1CON1bits.OCM = 0b110; // edge aligned PWM mode (high when OCxTMR = 0, low for OCxTMR = OCxRS)
    OC2CON1bits.OCM = 0b110;
    OC3CON1bits.OCM = 0b110; 
    OC4CON1bits.OCM = 0b110; 
    
    OC1CON2bits.SYNCSEL = 0x1F;
    OC2CON2bits.SYNCSEL = 0x1F;
    OC3CON2bits.SYNCSEL = 0x1F;
    OC4CON2bits.SYNCSEL = 0x1F;
    
    // set the period of the pwm at 10khz dividing fcy/pwm_freq
    OC1RS = 7200;
    OC2RS = 7200;
    OC3RS = 7200;
    OC4RS = 7200;
}

void pwm_move(int speed_rate, int yaw) {
    
    float left_motors = speed_rate - yaw/2.0f;
    float right_motors = speed_rate + yaw/2.0f;
    
    // Normalize PWM values to range -100% to 100%  
    float max_value = fmax(fabs(left_motors), fabs(right_motors));    
    if (max_value > 100.0f) {
        float scale = 100.0f / max_value;
        left_motors *= scale;
        right_motors *= scale;
    }
    
    left_motors = left_motors/100 * 7200;
    right_motors = right_motors/100 * 7200;
    
    
    OC1R = OC2R = OC3R = OC4R = 0;
 
    if (left_motors >= 0) {
        OC2R = left_motors; 
    } else {
        OC1R = -(left_motors);
    }
 
    if (right_motors >= 0) {
        OC4R = right_motors; 
    } else {
        OC3R = -(right_motors);
    }    
}

void pwm_stop(){
    OC1R = 0;
    OC2R = 0;
    OC3R = 0;
    OC4R = 0;
}