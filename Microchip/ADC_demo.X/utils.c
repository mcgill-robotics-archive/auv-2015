#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #include <p33Exxxx.h>
#endif

#include <string.h>

#include "utils.h"

#define UART_BAUD 115200
#define BRGVAL ((FCY/UART_BAUD)/16)-1

void configureClock(void);

void println(const char *line) {
    unsigned int size = strlen(line);
    while(size) {
        while(U1STAbits.UTXBF);     // wait while TX buffer full
        U1TXREG = *line;

        line++;
        size--;
    }
    while(U1STAbits.UTXBF);
    U1TXREG = '\n';                 // end line
    while(!U1STAbits.TRMT);         // wait for trasmit to finish
}

void configureUART(void) {
    configureClock();

    U1MODE = 0;
    U1STA = 0;
    U1BRG = BRGVAL;

    RPOR0bits.RP64R = 1;     // set RP64 as U1TX
    RPINR18bits.U1RXR = 75;  // RPI75 as U1RX

    U1MODEbits.UARTEN = 1;          // Enable UART
    U1STAbits.UTXEN = 1;            // Enable TX. Do this only after enable UART!
}

void configureClock(void) {
    PLLFBD = 98;                    // M = PLLFBD + 2
    CLKDIVbits.PLLPRE = 0;          // N2 = 2
    CLKDIVbits.PLLPOST = 0;         // N1 = 2
    // Initiate Clock Switch to FRC oscillator with PLL (NOSC=0b001)
    __builtin_write_OSCCONH(1);
    __builtin_write_OSCCONL(OSCCON | 1);
    // Wait for Clock switch to occur
    while (OSCCONbits.COSC!= 1);
    while(OSCCONbits.LOCK != 1);    // Clock Stabilization
}