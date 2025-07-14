#include "adc.h"

void adc_init() {
    AD1CON1bits.ADON = 0;
    AD1CON3bits.ADCS = 8; // TAD = Tcy * (8+1) = 125 ns
    AD1CON1bits.FORM = 0; // Output in integer format

    // automatic start - automatic conversion
    AD1CON1bits.ASAM = 1;
    AD1CON3bits.SAMC = 16; // sample time 16 Tad
    AD1CON1bits.SSRC = 7; // conversion starts after time specified by SAMC

    AD1CON1bits.AD12B = 0; // 10-bit mode

    //set_sequential_simultaneous
    AD1CON1bits.SIMSAM = 0;
    AD1CON2bits.SMPI = 1;

    // activate scan mode
    AD1CON2bits.CSCNA = 1;

    //set analog
    ANSELBbits.ANSB15 = 1;
    TRISBbits.TRISB15 = 1;
    ANSELBbits.ANSB11 = 1;
    TRISBbits.TRISB11 = 1;

    //add_analog_to_scan
    AD1CSSL = 0;
    AD1CSSLbits.CSS15 = 1;
    AD1CSSLbits.CSS11 = 1;

    AD1CON1bits.ADON = 1;
    AD1CON1bits.SAMP = 1;
}