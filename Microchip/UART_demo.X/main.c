#include <xc.h>
#include <string.h>

#include <libpic30.h>

#define INTERNAL_OSCILLATOR 7370000
#define FCY INTERNAL_OSCILLATOR * 100/8
#define UART_BAUD 115200
#define BRGVAL ((FCY/UART_BAUD)/16)-1

#define delay_s(d) { __delay32( (unsigned long) (((unsigned long long) d)*(FCY))); }
#define delay_ms(d) { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000ULL)); }
#define delay_us(d) { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000000ULL)); }
#define delay_ns(d) { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000000000ULL)); }

void configureClock(void);
void configureUART(void);
void println(const char *line);

// Select Internal FRC at POR
_FOSCSEL(FNOSC_FRC & IESO_OFF);
// Enable Clock Switching and Configure Primary Oscillator in XT mode
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);

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

void configureUART(void) {
    U1MODE = 0;
    U1STA = 0;
    U1BRG = BRGVAL;

    RPOR0bits.RP64R = 1;     // set RP64 as U1TX
    RPINR18bits.U1RXR = 75;  // RPI75 as U1RX

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