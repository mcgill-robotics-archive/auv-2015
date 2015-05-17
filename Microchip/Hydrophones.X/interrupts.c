#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #include <p33Exxxx.h>
#endif

#include "global.h"

//extern int passiveBuffer[4][1000];
//extern int burstBuffer[4][1000];

void configureInterrupts(void) {
    INTCON1bits.NSTDIS = 0;
    IPC3bits.AD1IP = 4;             // ADC interrupt priority = 4
    IFS0bits.AD1IF = 0;             // reset interrupt flag
    IEC0bits.AD1IE = 1;             // enable ADC interrupt
    //burstBuffer[0][0] = 1;
}

void __attribute__((interrupt, auto_psv)) _DMA1Interrupt(void) {
//    burstBuffer[0][0] = 1;
    IFS0bits.AD1IF = 0;             // reset interrupt flag
}