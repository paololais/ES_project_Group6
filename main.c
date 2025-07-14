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

// Defines the possible states of the system's operation mode
typedef enum {
    STATE_WAIT_FOR_START,
    STATE_MOVING,
    STATE_EMERGENCY
} State;

// Struct to group system state's variables
typedef struct {
    int ref_speed;      // user input: desired speed
    int ref_yawrate;    // user input: desired yaw rate
    int obstacle_dtc;   // flag: obstacle detected under threshold
    int time_elapsed;   // timer counter for obstacle absence duration
    int request_stop;   // event flag: request to stop motion (UART RX)
    int request_start;  // event flag: request to start motion (UART RX)
    State current_state; // current FSM state
    parser_state pstate; // parser state for UART command parsing
} SystemState;

// Struct for sensor data
typedef struct {
    int distance;
    AccSamples acc;
} SensorData;

// Global variables
volatile int btn_pressed = 0; // To track button RE8 pressed event
heartbeat schedInfo[MAX_TASKS];
volatile CircularBuffer cb_tx; // circular buffer for TX
volatile CircularBuffer cb_rx; // circular buffer for RX


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
// Reads distance from IR sensor
int read_IR(SystemState* sys_state){    
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
       
    sys_state->obstacle_dtc = (distance < DISTANCE_THRESHOLD) ? 1 : 0;
    
    return distance;
}

// Reads battery voltage
double read_battery(){
    AD1CON1bits.SAMP = 0;   // Stop sampling and start conversion
    while (!AD1CON1bits.DONE);  // // Wait until conversion is complete
    unsigned int adc_val = ADC1BUF0; // Read raw ADC value
    AD1CON1bits.SAMP = 1;
    
    // Convert ADC value to battery voltage (in volts)
    double battery_voltage = (adc_val / 1023.0) * 3.3 * 3.0;
    
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

// Interrupt UART RX
void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt() {
    char receivedChar = U1RXREG; // reads the received character
    cb_push(&cb_rx, receivedChar);
    IFS0bits.U1RXIF = 0; // Reset flag interrupt
}

// On entry Emergency state send: $MEMRG,1* 
// On exit send: $MEMRG,0*
void uartMEMRG(int emrg){
    char buffer[32];    
    sprintf(buffer, " $MEMRG,%d*\r\n", emrg);
    IEC0bits.U1TXIE = 0;
    for (int i = 0; i < strlen(buffer); i++) {
        cb_push(&cb_tx, buffer[i]);
    }
    IEC0bits.U1TXIE = 1;
}

// Acknowledgment message
// Positive ack if switch command received in Wait/Moving states: $MACK,1*
// Negative ack if switch command received in Emergency state: $MACK,0*
void uartMACK(int ack){
    char buffer[32]; 
    sprintf(buffer, " $MACK,%d*\r\n", ack);
    IEC0bits.U1TXIE = 0;
    for (int i = 0; i < strlen(buffer); i++) {
        cb_push(&cb_tx, buffer[i]);
    }
    IEC0bits.U1TXIE = 1;
}

// Processes incoming UART data from the receive buffer, parses complete messages,
// and updates the system state accordingly.
//
// For each received character:
// - Temporarily disables RX interrupt to safely pop a character from the circular buffer.
// - Passes the character to the message parser.
// - When a full message is recognized, handles different message types:
//   - "PCREF": extracts reference speed and yaw rate from the payload.
//   - "PCSTP": handles stop command, transitioning states unless in emergency,
//              sends acknowledgement via uartMACK.
//   - "PCSTT": handles start command, transitioning states unless in emergency,
//              sends acknowledgement via uartMACK.
//
// If an emergency state is detected during "PCSTP" or "PCSTT", sends negative ACK and breaks processing.
void processReceivedData(SystemState* sys_state) {
    char receivedChar;
    
    // If there are characters in the buffer
    while (!cb_is_empty(&cb_rx)) {
        //Critical region
        IEC0bits.U1RXIE = 0;   // Disable RX interrupt
        cb_pop(&cb_rx, &receivedChar); // Pop the character from the buffer
        IEC0bits.U1RXIE = 1;   // Enable RX interrupt
        
        int result = parse_byte(&sys_state->pstate, receivedChar);  
        if(result){
            if(!strcmp(sys_state->pstate.msg_type,"PCREF")){
                int i = 0;
                sys_state->ref_speed = extract_integer(sys_state->pstate.msg_payload);
                i = next_value(sys_state->pstate.msg_payload, i);
                sys_state->ref_yawrate = extract_integer(&sys_state->pstate.msg_payload[i]);
            }
            else if(!strcmp(sys_state->pstate.msg_type,"PCSTP")){
                if(sys_state->current_state == STATE_EMERGENCY){
                    uartMACK(0);    // Negative ACK
                }
                else if(sys_state->current_state == STATE_MOVING){
                    sys_state->request_stop = 1;
                }  
            }
            else if(!strcmp(sys_state->pstate.msg_type,"PCSTT")){
                if(sys_state->current_state == STATE_EMERGENCY){
                    uartMACK(0);    // Negative ACK
                }
                else if(sys_state->current_state == STATE_WAIT_FOR_START){
                    sys_state->request_start = 1;
                }                   
            }
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

// Formats the IR sensor distance data into a string and sends it over UART
// in the format "$MDIST,<distance>*"
void task_UartIR(SensorData* sensor_data){
    char buffer[32]; 
    sprintf(buffer, "$MDIST,%d*\r\n", sensor_data->distance);
    IEC0bits.U1TXIE = 0;
    for (int i = 0; i < strlen(buffer); i++) {
        cb_push(&cb_tx, buffer[i]);
    }
    IEC0bits.U1TXIE = 1;
}

// Reads the current battery voltage, formats it as a string with 2 decimal places,
// and sends it over UART in the format "$MBATT,xx.xx*".
void task_UartBatt(){
    char buffer[32];
    double battery_v = read_battery();
    
    sprintf(buffer, "$MBATT,%.2f*\r\n", battery_v);
    IEC0bits.U1TXIE = 0;
    for (int i = 0; i < strlen(buffer); i++) {
        cb_push(&cb_tx, buffer[i]);
    }
    IEC0bits.U1TXIE = 1;
}

// Reads a new accelerometer measurement, updates a buffer of the last 5 samples,
// computes the average for each axis, and sends the averaged data over UART
// in the format "$MACC,x_avg,y_avg,z_avg*".
void task_UartAcc(AccSamples* acc){
        
    Data values = read_acc(); // Get latest accelerometer reading
    
    acc->x[acc->index] = values.x;
    acc->y[acc->index] = values.y;
    acc->z[acc->index] = values.z;
    
    acc->index++;
    if (acc->index == 5){
        acc->index = 0;
    }
    
    // Compute average of last 5 samples
    int x_avg = 0;
    int y_avg = 0;
    int z_avg = 0;
    
    for(int i = 0; i < 5; i++){
        x_avg += acc->x[i];
        y_avg += acc->y[i];
        z_avg += acc->z[i];
    }
    
    x_avg = x_avg / 5;   
    y_avg = y_avg / 5;
    z_avg = z_avg / 5;
    
    // Format and send data over UART
    char buffer[32];
    sprintf(buffer, "$MACC,%d,%d,%d*\r\n", x_avg, y_avg, z_avg);
    IEC0bits.U1TXIE = 0;
    for (int i = 0; i < strlen(buffer); i++) {
        cb_push(&cb_tx, buffer[i]);
    }
    IEC0bits.U1TXIE = 1;
}


/**
 * FSM - State Machine for system operation
 * 
 * Handles transitions between three states:
 * - WAIT_FOR_START: waits for button press to start moving
 * - MOVING: moves with set speed and yaw rate; transitions to emergency if obstacle detected or stop if button pressed
 * - EMERGENCY: stops motion, blinks lights; returns to WAIT_FOR_START after obstacle cleared for 5 seconds
 * 
 * Also manages PWM commands, button RE8 interrupt enable/disable and UART notifications.
 */
void FSM(SystemState* sys_state) {
    switch (sys_state->current_state) {
        case STATE_WAIT_FOR_START:             
            // if button RE8 has been pressed: transition to "Moving" state
            if(btn_pressed){
                btn_pressed = 0; // reset event
                sys_state->current_state = STATE_MOVING;
            }
            // if user sent "$PCSTT,*" via UART
            else if (sys_state->request_start) {
                sys_state->request_start = 0;
                sys_state->current_state = STATE_MOVING;
                uartMACK(1); // Positive ACK
            }
            else {
                // No motion
                pwm_move(0,0);
            }
            
            break;

        case STATE_MOVING:            
            // if button RE8 has been pressed: transition to "Wait for start" state
            if(btn_pressed){
                btn_pressed = 0; // reset event
                sys_state->current_state = STATE_WAIT_FOR_START;
            }
            // if user sent "$PCSTP,*" via UART
            else if (sys_state->request_stop) {
                sys_state->request_stop = 0;
                sys_state->current_state = STATE_WAIT_FOR_START;
                uartMACK(1); // Positive ACK
            }            
            // if obstacle detected under threshold: transition to "Emergency" state
            else if(sys_state->obstacle_dtc){
                sys_state->current_state = STATE_EMERGENCY;
                schedInfo[1].enable = 1; // enable lights blinking                
                IEC1bits.INT1IE = 0; // No input from button RE8 -> disable interrupt INT1E
                uartMEMRG(1); // send UART message
            }
            else {
                // Motion
                pwm_move(sys_state->ref_speed, sys_state->ref_yawrate);                
            }            
            break;

        case STATE_EMERGENCY:                        
            // if no obstacle detected under threshold for more than 5s: 
            // transition to "Wait for start" state
            if(!sys_state->obstacle_dtc){
                sys_state->time_elapsed++;                
                if(sys_state->time_elapsed >= 2500){
                    sys_state->time_elapsed = 0;
                    schedInfo[1].enable = 0; // Disable lights blinking
                    LATBbits.LATB8 = 0; LATFbits.LATF1 = 0; // switch off lights
                    uartMEMRG(0); // send UART message
                    
                    sys_state->current_state = STATE_WAIT_FOR_START;
                    IFS1bits.INT1IF = 0; // Reset interrupt flag
                    IEC1bits.INT1IE = 1; // Re enable interrupt for RE8                    
                    break;
                }
            } else {
                // Obstacle encountered: reset time_elapsed to zero
                sys_state->time_elapsed = 0; 
                // No motion
                pwm_move(0,0);  
            }            
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
    
  
    // System state initialization
    SystemState sys_state = {0};
    SensorData sensor_data = {0};    
    sys_state.current_state = STATE_WAIT_FOR_START;
    sys_state.pstate.state = STATE_DOLLAR;
    
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
    schedInfo[2].params = &sensor_data.distance;
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
    schedInfo[4].params = &sensor_data.acc;
    schedInfo[4].enable = 1;
    
    // Initialize devices
    pwm_init();
    adc_init();
    UART1_Init();
    spi_init();
    acc_enable(); // Enable accelerometer
    
    // Circular buffers initialization
    cb_init(&cb_tx);
    cb_init(&cb_rx);    
    
    // Control loop frequency 500 Hz (2ms))
    tmr_setup_period(TIMER1, 2);
    
    while(1){
        // IR sensor reading, crucial for the control loop and FSM
         sensor_data.distance = read_IR(&sys_state);
        
        // FSM function handles transitions between states and actions
        FSM(&sys_state);
        
        // Schedule periodic enabled tasks
        scheduler(schedInfo, MAX_TASKS);
        
        // Handle incoming UART messages
        processReceivedData(&sys_state);
        
        tmr_wait_period(TIMER1);
    }
    
    return 0;
}
