#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #include <p33Exxxx.h>
#endif

#include <stdio.h>
#include <limits.h>

#include "utils.h"

void configureADC(void);
int analogRead(void);

int main(void) {
    configureUART();
    configureADC();

    delay_s(4);

    int buffer[1000];

    int i = 0;
    for(i = 0; i < 1000; i++) {
        buffer[i] = analogRead();
    }

    for(i = 0; i < 1000; i++) {
        println(printf("%d,", buffer[i]));
    }

    while(1);
    return 0;
}

void configureADC(void) {
    ANSELBbits.ANSB0 = 1;   // set RB3 (AN5) to analog
    TRISBbits.TRISB0 = 1;   // set RB3 as an input

    AD1CON1 = 0;    // disable ADC before configuration

    AD1CON1bits.AD12B = 1;   // 1 channel
    AD1CON1bits.SSRC = 7;   // internal counter ends sampling and starts conversion
    AD1CON2 = 0;            // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
    AD1CON3 = 0x0f01;       // TAD = 4*TPB, acquisition time = 15*TAD

    AD1CON1bits.ADON = 1;    // Enable ADC
}

int analogRead(void) {
    AD1CON1bits.SAMP = 1;           // Begin sampling
    while(AD1CON1bits.SAMP);        // wait until acquisition is done
    while(!AD1CON1bits.DONE);       // wait until conversion done

    return ADC1BUF0;                // result stored in ADC1BUF0
}