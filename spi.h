/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef SPI_H
#define	SPI_H

#include <xc.h> // include processor files - each processor file is guarded.
#include "timer.h"
#define ACC_CS LATBbits.LATB3
#define MAG_CS LATDbits.LATD6
#define GYR_CS LATBbits.LATB4

// Structure to store a single accelerometer measurement (x, y, z axes)
typedef struct {
    int x;
    int y;
    int z;
} Data;

// Structure to store the last 5 accelerometer measurements for each axis (x, y, z).
// Used to compute an average. 'index' tracks the current write position;
// reset to 0 when it reaches 5.
typedef struct {
    int x[5];
    int y[5];
    int z[5];
    int index;
} AccSamples;

// Initializes SPI communication
void spi_init();

//  Writes the data to the SPI and returns the value read from the other device
unsigned int spi_write(unsigned int data);

// Enables the accelerometer by writing a configuration value to address 0x10 via SPI.
// Also clears the SPI overflow flag if set.
void acc_enable();

// Converts accelerometer values into a single 16-bit integer
int convert_acc(unsigned int msb, unsigned int lsb);

// Reads accelerometer data, collects them in a struct and returns it
Data read_acc();

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* SPI_H */

