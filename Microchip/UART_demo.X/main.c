#include <xc.h>
#include <string.h>
#include <libpic30.h>

#define FCY 46875000UL
#define UART_BAUD 115200
#define BRGVAL ((FCY/UART_BAUD)/16)-1
#define delay_ms(d) { __delay32( (unsigned long) ((d)*(FCY)/1000UL)); }

void configureClock(void);
void configureUART(void);
void println(const char *line);

int main(void) {
    configureClock();
    configureUART();
    delay_ms(100);

    while(1) {
        println("Hello world!");
        delay_ms(10);
    }

    return 0;
}

void configureClock(void) {
    /*
     * This sets FCY  to 46875000
     * I don't know why.
     * I don't know how.
     * Don't change it!
     */
    PLLFBD = 48;                    // M = 50
    CLKDIVbits.PLLPRE = 0;          // N2 = 2
    CLKDIVbits.PLLPOST = 0;         // N1 = 2
    while(OSCCONbits.LOCK != 1);    // Clock Stabilization
}

void configureUART(void) {
    U1MODE = 0;
    U1STA = 0;
    U1BRG = BRGVAL;

    RPOR0bits.RP64R = 0b000001;     // set RP64 as U1TX
    RPINR18bits.U1RXR = 0b1001011;  // RPI75 as U1RX

    U1MODEbits.UARTEN = 1;          // Enable UART
    U1STAbits.UTXEN = 1;            // Enable TX. Do this only after enable UART!
}

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