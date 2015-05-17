#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #include <p33Exxxx.h>
#endif

#include <string.h>

#define FCY 46875000UL
#define UART_BAUD 9600
#define BRGVAL ((FCY/UART_BAUD)/16)-1
#define delay_ms(d) { __delay32( (unsigned long) ((d)*(FCY)/1000UL)); }

#pragma config JTAGEN = OFF
#pragma config FWDTEN = OFF

// Select Internal FRC at POR
_FOSCSEL(FNOSC_FRC & IESO_OFF);
// Enable Clock Switching and Configure Primary Oscillator in XT mode
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);

void configureClock(void);
void configureUART(void);
void println(const char *line);

int main(void) {
    configureClock();
    configureUART();
    delay_ms(100);

    TRISDbits.TRISD1 = 0;
    LATDbits.LATD1 = 1;

    while(1) {
        println("Hello world!");
        delay_ms(10);
    }

    return 0;
}

void configureClock(void) {
    /*
     * This sets FCY  to 46875000
     */
    PLLFBD = 48;                    // M = 50
    CLKDIVbits.PLLPRE = 0;          // N2 = 2
    CLKDIVbits.PLLPOST = 0;         // N1 = 2
    // Initiate Clock Switch to FRC oscillator with PLL (NOSC=0b001)
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(OSCCON | 0x01);
    // Wait for Clock switch to occur
    while (OSCCONbits.COSC!= 0b001);
    // Wait for PLL to lock
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