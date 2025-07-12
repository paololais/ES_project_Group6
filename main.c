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
#include "spi.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

#define DISTANCE_THRESHOLD 15 // 15 CM
#define MAX_TASKS 5
heartbeat schedInfo[MAX_TASKS];

// Global variables to handle events for FSM
// To track button pressed event to enable transition
// between Wait and Moving states
volatile int btn_pressed = 0; 

// To track if obstacle has been detected under threshold
volatile int obstacle_dtc = 0;
// To track time no obstacle has been detected
// if time >= 2500 (5s = 5000ms = 2500 Heartbeat)
volatile int time_elapsed = 0;

volatile CircularBuffer cb_tx;
char buffer[32];

//Button RE8
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
      btn_pressed = 1;
  }
}

// ADC
// read distance from IR sensor
int read_IR(){    
    AD1CON1bits.SAMP = 0;
    while (!AD1CON1bits.DONE);
    unsigned int adc_val = ADC1BUF1; // Raw ADC value
    AD1CON1bits.SAMP = 1;
    double voltage = (adc_val / 1023.0) * 3.3;
    
    double v2 = voltage * voltage;
    double v3 = v2 * voltage;
    double v4 = v3 * voltage;
    
    double raw_distance = (2.34 - 4.74 * voltage + 4.06 * v2 - 1.60 * v3 + 0.24 * v4) * 100;
    int distance = (int)round(raw_distance);
       
    obstacle_dtc = (distance < DISTANCE_THRESHOLD) ? 1 : 0;
    
    return distance;
}

// read battery voltage
int read_battery(){
    AD1CON1bits.SAMP = 0;
    while (!AD1CON1bits.DONE);
    unsigned int adc_val = ADC1BUF0; // Raw ADC value
    AD1CON1bits.SAMP = 1;
    double battery_voltage = (adc_val / 1023.0) * 3.3 / 3.0;
    
    return battery_voltage;
}

// UART
// Interrupt UART TX
void __attribute__((__interrupt__, __auto_psv__)) _U1TXInterrupt() {
    IFS0bits.U1TXIF = 0; // Clear the TX interrupt flag
    
    char c;
    
    while(U1STAbits.UTXBF == 0){
        // If there are characters in the TX buffer, send them
        if (!cb_is_empty(&cb_tx)) {
            cb_pop(&cb_tx, &c); // Pop a character from the TX buffer
            U1TXREG = c;        // Write the character to the UART TX register
        } else {
            IEC0bits.U1TXIE = 0;
            break;
        }
    }
}

// PERIODIC TASKS
void task_blink_led(){
    LATAbits.LATA0 = !LATAbits.LATA0;
}

void task_blink_lights(){
    LATBbits.LATB8 = !LATBbits.LATB8;
    LATFbits.LATF1 = !LATFbits.LATF1;
}

void task_UartIR(int* distance){
    sprintf(buffer, "$MDIST,%d*\r\n", *distance);
    IEC0bits.U1TXIE = 0;
    for (int i = 0; i < strlen(buffer); i++) {
        cb_push(&cb_tx, buffer[i]);
    }
    IEC0bits.U1TXIE = 1;
}

void task_UartBatt(){
    double battery_v = read_battery();
    
    sprintf(buffer, "$MBATT,%.2f*\r\n", battery_v);
    IEC0bits.U1TXIE = 0;
    for (int i = 0; i < strlen(buffer); i++) {
        cb_push(&cb_tx, buffer[i]);
    }
    IEC0bits.U1TXIE = 1;
}

/*
void task_UartAcc(ACCavg* avg){
    int x_avg = 0;
    int y_avg = 0;
    int z_avg = 0;
        
    Values values = read_acc();
    
    avg->x[avg->sample_counter] = values.valuex;
    avg->y[avg->sample_counter] = values.valuey;
    avg->z[avg->sample_counter] = values.valuez;
    
    avg->sample_counter++;
    if (avg->sample_counter == 5){
        avg->sample_counter = 0;
    }
    
    for(int i = 0; i < 5; i++){
        x_avg += avg->x[i];
        y_avg += avg->y[i];
        z_avg += avg->z[i];
    }
    
    x_avg = x_avg / 5;   
    y_avg = y_avg / 5;
    z_avg = z_avg / 5;
    
    sprintf(buffer, "$MACC,%d,%d,%d*\r\n", x_avg, y_avg, z_avg);
    IEC0bits.U1TXIE = 0;
    for (int i = 0; i < strlen(buffer); i++) {
        cb_push(&cb_tx, buffer[i]);
    }
    IEC0bits.U1TXIE = 1;
}
 */

void task_UartAcc(ACCavg* avg){
    Values values = read_acc();
    
    sprintf(buffer, "$MACC,%d,%d,%d*\r\n", values.valuex, values.valuey, values.valuez);
    IEC0bits.U1TXIE = 0;
    for (int i = 0; i < strlen(buffer); i++) {
        cb_push(&cb_tx, buffer[i]);
    }
    IEC0bits.U1TXIE = 1;
}

// STATE MACHINE
typedef enum {
    STATE_WAIT_FOR_START,
    STATE_MOVING,
    STATE_EMERGENCY
} State;

void FSM(State *currentState) {
    switch (*currentState) {
        case STATE_WAIT_FOR_START:             
            // if button RE8 has been pressed: transition to "Moving" state
            if(btn_pressed){
                btn_pressed = 0; // reset event
                *currentState = STATE_MOVING;
                break;
            }
            
            // No motion
            pwm_move(0,0);
            
            break;

        case STATE_MOVING:            
            // if button RE8 has been pressed: transition to "Wait for start" state
            if(btn_pressed){
                btn_pressed = 0; // reset event
                *currentState = STATE_WAIT_FOR_START;    
                break;
            }
            
            // if obstacle detected under threshold: transition to "Emergency" state
            if(obstacle_dtc){
                *currentState = STATE_EMERGENCY;
                schedInfo[1].enable = 1; // enable lights blinking
                // No input from button RE8 -> disable interrupt INT1E
                IEC1bits.INT1IE = 0;
                break;
            }
            
            // Motion
            pwm_move(60,30); //example, this should have user input as parameters
            
            break;

        case STATE_EMERGENCY:                        
            // if no obstacle detected under threshold for more than 5s: 
            // transition to "Wait for start" state
            if(!obstacle_dtc){
                time_elapsed++;                
                if(time_elapsed >= 2500){
                    time_elapsed = 0;
                    schedInfo[1].enable = 0; // Disable lights blinking
                    LATBbits.LATB8 = 0; LATFbits.LATF1 = 0; // switch off lights
                    
                    *currentState = STATE_WAIT_FOR_START;
                    IFS1bits.INT1IF = 0; // Reset interrupt flag
                    IEC1bits.INT1IE = 1; // Re enable interrupt for RE8
                    break;
                }
            } else {
                // Obstacle encountered: reset time_elapsed to zero
                time_elapsed = 0;
            }            
            
            // No motion
            pwm_move(0,0);            
            
            break;
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
    
    // LED A0
    TRISAbits.TRISA0 = 0; // LED output
    LATAbits.LATA0 = 0; // switch off LED at the beginning
    
    // RB8 Left side lights - RF1 Right-side lights
    TRISBbits.TRISB8 = 0; LATBbits.LATB8 = 0;
    TRISFbits.TRISF1 = 0; LATFbits.LATF1 = 0;
    
    // Variables for tasks parameters
    int distance = 0; 
    ACCavg avg;
    avg.sample_counter = 0;
    
    // Scheduler configuration        
    // Led blink
    schedInfo[0].n = 0;
    schedInfo[0].N = 250;
    schedInfo[0].f = (void (*)(void *))task_blink_led;
    schedInfo[0].params = NULL;
    schedInfo[0].enable = 1;
    
    // Left and right lights blink
    schedInfo[1].n = 0;
    schedInfo[1].N = 250;
    schedInfo[1].f = (void (*)(void *))task_blink_lights;
    schedInfo[1].params = NULL;
    schedInfo[1].enable = 0;    // Enable only in Emergency state
    
    // UART IR message TX - 100Hz
    schedInfo[2].n = 0;
    schedInfo[2].N = 50;
    schedInfo[2].f = (void (*)(void *))task_UartIR;
    schedInfo[2].params = &distance;
    schedInfo[2].enable = 1;
    
    // UART Battery message TX - 1Hz
    schedInfo[3].n = 0;
    schedInfo[3].N = 500;
    schedInfo[3].f = (void (*)(void *))task_UartBatt;
    schedInfo[3].params = NULL;
    schedInfo[3].enable = 1;
    
    // UART Accelerometer message TX - 100Hz
    schedInfo[4].n = 0;
    schedInfo[4].N = 50;
    schedInfo[4].f = (void (*)(void *))task_UartAcc;
    schedInfo[4].params = &avg;
    schedInfo[4].enable = 1;
    
    // Initialize devices
    pwm_init();
    adc_init();
    UART1_Init();
    spi_init();
    
    // Enable SPI accelerometer
    acc_enable();
    
    // Circular buffers initialization
    cb_init(&cb_tx);
    //cb_init(&cb_rx);
    
    // Current state initialization for FSM
    State currentState = STATE_WAIT_FOR_START;
    
    // Control loop frequency 500 Hz (2ms))
    tmr_setup_period(TIMER1, 2);
    
    while(1){
        // IR sensor reading, crucial for the control loop and FSM
        distance = read_IR();
        
        // FSM function handles transitions between states and actions
        FSM(&currentState);
                
        scheduler(schedInfo, MAX_TASKS);
        
        tmr_wait_period(TIMER1);
    }
    
    return 0;
}
