#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #include <p33Exxxx.h>
#endif

#include <stdlib.h>

#include "interrupts.h"
#include "util.h"
#include "buffers.h"

int adcInterruptCounter = 0;
int timerInterruptCounter = 0;

void configureInterrupts(void) {
    INTCON1bits.NSTDIS = 0;         // Interrupt nesting enabled

    // ADC interupt configuration
    IPC3bits.AD1IP = 4;             // ADC interrupt priority = 4
    IFS0bits.AD1IF = 0;             // reset interrupt flag
    IEC0bits.AD1IE = 1;             // enable ADC interrupt

    // Timer 1 interupt configuration
    IPC0bits.T1IP = 3;              // Timer 1 interrupt priority = 3
    IFS0bits.T1IF = 0;              // reset interrupt flag
    IEC0bits.T1IE = 1;              // enable Timer 1 interrupt
}

// Interrupts every fourth conversion on ADC
void __attribute__((interrupt, no_auto_psv)) _AD1Interrupt(void) {

    if(adcInterruptCounter >= 1000) { //limit to 1000 samples before transmiting
        adcInterruptCounter = 0;
        setBufferStatus(1);
    }

    if(sample(1) > THRESHOLD || sample(2) > THRESHOLD ||
            sample(3) > THRESHOLD || sample(4) > THRESHOLD || adcInterruptCounter > 0) {

        store(sample(1), 1, adcInterruptCounter, BURST);
        store(sample(2), 2, adcInterruptCounter, BURST);
        store(sample(3), 3, adcInterruptCounter, BURST);
        store(sample(4), 4, adcInterruptCounter, BURST);
        adcInterruptCounter++;
    }

    IFS0bits.AD1IF = 0;             // reset interrupt flag
}

// should interrupt every 5.5 us
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    

    IFS0bits.T1IF = 0;              // reset interrupt flag
}