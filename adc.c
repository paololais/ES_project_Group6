#include "adc.h"

void adc_init1() {
    // ADC config
    // Disable all analog inputs first
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    
    // Enable AN11 (RB11) as analog input
    ANSELBbits.ANSB11 = 1;
    
    AD1CON1bits.ADON = 0;       // Turn off ADC to configure
    AD1CON1bits.AD12B = 0;      // 10-bit mode
    AD1CON1bits.FORM = 0;       // Integer output
    AD1CON1bits.SSRC = 0;       // Manual conversion trigger
    AD1CON1bits.ASAM = 0;       // Manual sampling

    AD1CON2bits.CHPS = 0;       // Use CH0 only
    AD1CON3bits.ADRC = 0;       // Use system clock
    AD1CON3bits.SAMC = 0;       // Not used for manual sampling
    AD1CON3bits.ADCS = 8;       // ADC conversion clock select (Tad)

    AD1CHS0bits.CH0SA = 11;     // Select AN11 as input

    AD1CON1bits.ADON = 1;       // Turn on ADC
}

void adc_init2() {
    // ADC config
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;

    ANSELBbits.ANSB5 = 1;
    TRISBbits.TRISB5 = 1; // Imposta RB5 come input

    AD1CON1bits.ADON = 0;       // Turn off ADC to configure
    AD1CON1bits.AD12B = 0;      // 10-bit mode
    AD1CON1bits.FORM = 0;       // Integer output
    AD1CON1bits.SSRC = 7;       // Auto conversion after sampling
    AD1CON1bits.ASAM = 0;       // Manual sampling

    AD1CON2bits.CHPS = 0;       // Use CH0 only
    AD1CON3bits.ADRC = 0;       // Use system clock
    AD1CON3bits.SAMC = 16;       // Not used for manual sampling
    AD1CON3bits.ADCS = 8;       // ADC conversion clock select (Tad)

    //AD1CHS0bits.CH0SA = 0;     // Select AN11 as input
    AD1CHS0bits.CH0SA = 5; // Seleziona AN4 (RB4)

    AD1CON1bits.ADON = 1;       // Turn on ADC
}
void adc_init3() {
    // ADC config
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    
    //Enable AN5 as analog input
    ANSELBbits.ANSB5 = 1;
    TRISBbits.TRISB5 = 1; // Imposta RB5 come input
    
    //Enable AN11 (RB11) as analog input
    ANSELBbits.ANSB11 = 1;

    AD1CON1bits.ADON = 0;       // Turn off ADC to configure
    AD1CON1bits.AD12B = 0;      // 10-bit mode
    AD1CON1bits.FORM = 0;       // Integer output
    AD1CON1bits.SSRC = 7;       // Auto conversion after sampling
    AD1CON1bits.ASAM = 1;       // Manual sampling

    AD1CON2bits.CHPS = 0;       // Use CH0 only
    AD1CON3bits.ADRC = 0;       // Use system clock
    AD1CON3bits.SAMC = 16;       // Not used for manual sampling
    AD1CON3bits.ADCS = 8;       // ADC conversion clock select (Tad)

    AD1CHS0bits.CH0SA = 11;     // Select AN11 as input
    AD1CHS0bits.CH0SA = 5;      // Seleziona AN4 (RB4)

    AD1CON1bits.ADON = 1;       // Turn on ADC
}

// ADC initialization for scan mode automatic sampling
void adc_init3_v2() {
    // Disable all analog inputs first
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    
    // Enable AN5 (RB5) for IR sensor
    ANSELBbits.ANSB5 = 1;
    TRISBbits.TRISB5 = 1;
    
    // Enable AN11 (RB11) for battery voltage
    ANSELBbits.ANSB11 = 1;
    TRISBbits.TRISB11 = 1;
    
    // Configure ADC
    AD1CON1bits.ADON = 0;      // Turn off ADC to configure
    AD1CON1bits.AD12B = 0;     // 10-bit mode
    AD1CON1bits.FORM = 0;      // Integer output
    AD1CON1bits.SSRC = 2;      // Timer3 triggers conversion
    AD1CON1bits.ASAM = 1;      // Automatic sampling starts after conversion
    
    AD1CON2bits.CSCNA = 1;     // Enable scan mode
    AD1CON2bits.CHPS = 0;      // Use CH0 only
    AD1CON2bits.SMPI = 1;      // 2 samples per interrupt (0 = 1 sample, 1 = 2 samples)
    
    AD1CON3bits.ADRC = 0;      // Use system clock
    AD1CON3bits.SAMC = 10;     // Auto sample time
    AD1CON3bits.ADCS = 8;      // ADC conversion clock select
    
    // Configure scan channels
    AD1CSSLbits.CSS5 = 1;      // Include AN5 (IR sensor) in scan
    AD1CSSLbits.CSS11 = 1;     // Include AN11 (battery voltage) in scan
    
    // Enable ADC interrupt
    IFS0bits.AD1IF = 0;        // Clear ADC interrupt flag
    IEC0bits.AD1IE = 1;        // Enable ADC interrupt
    
    AD1CON1bits.ADON = 1;      // Turn on ADC
}