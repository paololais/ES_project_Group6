#include "adc.h"

void adc_init() {
    AD1CON1bits.ADON = 0;
    AD1CON3bits.ADCS = 8;
    AD1CON1bits.FORM = 0; // Integer format for output

    // Automatic start - automatic conversion
    AD1CON1bits.ASAM = 1;
    AD1CON3bits.SAMC = 16; // Sample time
    AD1CON1bits.SSRC = 7;

    AD1CON1bits.AD12B = 0; // 10-bit mode

    AD1CON1bits.SIMSAM = 0;
    AD1CON2bits.SMPI = 1;

    // Activate scan mode
    AD1CON2bits.CSCNA = 1;

    // Set analog
    ANSELBbits.ANSB15 = 1;
    TRISBbits.TRISB15 = 1;
    ANSELBbits.ANSB11 = 1;
    TRISBbits.TRISB11 = 1;
    
    // Configure scan channels
    AD1CSSL = 0;    
    AD1CSSLbits.CSS15 = 1;
    AD1CSSLbits.CSS11 = 1;

    AD1CON1bits.ADON = 1;
    AD1CON1bits.SAMP = 1;
}