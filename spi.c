/*
 * File:   spi.c
 * Author: paolo
 *
 * Created on 31 marzo 2025, 10.29
 */

#include "xc.h"
#include "spi.h"

void spi_init() {
    SPI1STATbits.SPIEN = 0;    // Disable SPI to configure it

    SPI1CON1bits.MSTEN = 1;    // Master mode
    SPI1CON1bits.MODE16 = 0;   // 8-bit mode
    SPI1CON1bits.CKE = 1;      // Data changes on transition from idle to active clock state
    SPI1STATbits.SPIROV = 0;   // Clear overflow

    // Fcy = 72MHz, F_SPI = Fcy / (Primary * Secondary)
    SPI1CON1bits.PPRE = 0b00;  // Primary prescaler 64:1
    SPI1CON1bits.SPRE = 0b101; // Secondary prescaler 3:1
    
    // Remapping configuration
    TRISAbits.TRISA1 = 1; // RA1-RPI17 MISO
    TRISFbits.TRISF12 = 0; // RF12-RP108 SCK
    TRISFbits.TRISF13 = 0; // RF13-RP109 MOSI
    
    // configure CS pins
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB4 = 0;
    TRISDbits.TRISD6 = 0;
    
    RPINR20bits.SDI1R = 0b0010001; // MISO (SDI1) - RPI17
    RPOR12bits.RP109R = 0b000101; // MOSI (SDO1) - RF13;
    RPOR11bits.RP108R = 0b000110; // SCK1; 
    
    ACC_CS = 1;
    GYR_CS = 1;
    MAG_CS = 1;

    SPI1STATbits.SPIEN = 1;    // enable SPI
}

unsigned int spi_write(unsigned int data){
    unsigned int value = 0;
    
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = data;
    while (SPI1STATbits.SPIRBF == 0);
    value = SPI1BUF;
    
    return value;
}

void acc_enable(){
    unsigned int addr = 0x0010;
    unsigned int value = 0b00001100;
    
    ACC_CS = 0;
    spi_write(addr);
    spi_write(value);
    ACC_CS = 1;
    
    //clear overflow
    if (SPI1STATbits.SPIROV == 1){
        SPI1STATbits.SPIROV = 0;
    }
}

int convert_acc(unsigned int msb, unsigned int lsb){
    int value = 0;
    
    msb = msb & 0x00F0;
    lsb = (int) lsb << 8; // left shift by 8
    value = (int) (lsb | msb) >> 4; // put together the two bytes and right shift by 4
    
    return value;
}

Data read_acc() {
    Data acc;
    unsigned int value = 0x00;
    unsigned int value1;
    unsigned int value2;
    unsigned int read_addr = 0x0002 | 0x80;
    
    ACC_CS = 0;
    spi_write(read_addr);
    
    value1 = spi_write(value);
    value2 = spi_write(value);
    acc.x = convert_acc(value1, value2);
    
    value1 = spi_write(value);
    value2 = spi_write(value);
    acc.y = convert_acc(value1, value2);
    
    value1 = spi_write(value);
    value2 = spi_write(value);
    acc.z = convert_acc(value1, value2);    
    ACC_CS = 1;
    
    if (SPI1STATbits.SPIROV == 1){
        SPI1STATbits.SPIROV = 0;
    }
    
    return acc;
}