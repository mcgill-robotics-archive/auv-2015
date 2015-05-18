#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #include <p33Exxxx.h>
#endif

#include "config.h"
#include "util.h"
#include "interrupts.h"

#define UART_BAUD 115200
#define BRGVAL ((FCY/UART_BAUD)/16)-1

// Select Internal FRC at POR
_FOSCSEL(FNOSC_FRC & IESO_OFF);
// Enable Clock Switching and Configure Primary Oscillator in XT mode
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);

void initApp(void) {
    configureClock();
    configureUART();
    println("[+] configured Clock & UART");
    configureADC();
    println("[+] configured ADC");
    //configureInterrupts();
}

void configureClock(void) {
    PLLFBD = M - 2;
    CLKDIVbits.PLLPRE = N2 - 2;
    CLKDIVbits.PLLPOST = N1 - 2;
    // Initiate Clock Switch to FRC oscillator with PLL (NOSC=0b001)
    __builtin_write_OSCCONH(1);
    __builtin_write_OSCCONL(OSCCON | 1);
    // Wait for Clock switch to occur
    while(OSCCONbits.COSC!= 1);
    while(OSCCONbits.LOCK != 1);    // Clock Stabilization
}

void configureUART(void) {
    U1MODE = 0;
    U1STA = 0;
    U1BRG = BRGVAL;

    RPOR0bits.RP64R = 1;            // set RP64 as U1TX
    RPINR18bits.U1RXR = 75;         // set U1RX to RPI75

    U1MODEbits.UARTEN = 1;          // Enable UART
    U1STAbits.UTXEN = 1;            // Enable TX. Do this only after enable UART!
}

void configureADC(void) {
    //set AN0-AN3 to analogue
    ANSELBbits.ANSB0 = 1;
    ANSELBbits.ANSB1 = 1;
    ANSELBbits.ANSB2 = 1;
    ANSELBbits.ANSB3 = 1;

    //set AN0-AN3 to input
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 1;

    //ADC Configuration register 1
    AD1CON1bits.ADON = 0;           // Disable ADC for configuration
    AD1CON1bits.AD12B = 0;          // 10bit, 4Channel ADC operation
    AD1CON1bits.FORM = 0;           // Output unsigned integer
    AD1CON1bits.SSRC = 7;           // internal counter ends sampling and starts conversion
    AD1CON1bits.SIMSAM = 1;         // Simultaneous channel sampling based on "CHPS"
    AD1CON1bits.ASAM = 1;           // Begin sampling immediatly after conversion

    //ADC Configuration register 2
    AD1CON2bits.CHPS = 3;           // Convert Ch0, Ch1, Ch2, Ch3
    AD1CON2bits.SMPI = 3;           // Restart buffer after every 4th conversion

    //ADC Configuration register 3
    AD1CON3bits.ADRC = 0;           // Uses Systems Clock
    AD1CON3bits.SAMC = 0;           // Auto Sample Time of 0
    AD1CON3bits.ADCS = 0;           // ADC convertion clock
                                    // TAD (ADC period) = 1 * TCY (Clock Period)

    //Channels 1,2,3 pin configuration
    AD1CHS123bits.CH123SA = 0;      // Sets positive ends of inputs to AN1, AN2, AN0
    AD1CHS123bits.CH123NA = 0;      // Ground negative end

    //Channel 0 pin configuration
    AD1CHS0bits.CH0SA = 3;          // Sets positive end of input to AN3
    AD1CHS0bits.CH0NA = 0;          // Ground negative end

    //Activate ADC
    AD1CON1bits.ADON = 1;
}