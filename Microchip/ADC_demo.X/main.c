#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #include <p33Exxxx.h>
#endif

#include <stdio.h>
#include <limits.h>

#include "utils.h"

//#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
//#pragma config JTAGEN = OFF         // JTAG Enable bit (JTAG is disabled)
//
//// FPOR
//#pragma config BOREN = ON           // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
//#pragma config ALTI2C1 = OFF        // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
////#pragma config ALTI2C2 = OFF        // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
//
//// FWDT
//#pragma config WDTPOST = PS32768    // Watchdog Timer Postscaler bits (1:32,768)
//#pragma config WDTPRE = PR128       // Watchdog Timer Prescaler bit (1:128)
//#pragma config PLLKEN = ON          // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
//#pragma config WINDIS = OFF         // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
//#pragma config FWDTEN = OFF         // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)
//
//// FOSC
//#pragma config POSCMD = XT          // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
//#pragma config OSCIOFNC = OFF       // OSC2 Pin Function bit (OSC2 is clock output)
//#pragma config IOL1WAY = OFF        // Peripheral pin select configuration (Allow multiple reconfigurations)
//#pragma config FCKSM = CSECMD       // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)
//
//// FOSCSEL
//#pragma config FNOSC = FRC          // Oscillator Source Selection (Internal Fast RC (FRC))
//#pragma config IESO = OFF           // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)
//
//// FGS
//#pragma config GWRP = OFF           // General Segment Write-Protect bit (General Segment may be written)
//
//// User Defines
//#define FCY         40000000        // User must calculate and enter FCY here
//#define Dly_Time    ( 20E-6 * FCY ) // ADC Off-to-On delay
//
////uncomment next line if __eds__ is supported
////#define _HAS_DMA_
//#define NUMSAMP 256
//
//#ifdef _HAS_DMA_
//    __eds__ int bufferA[NUMSAMP] __attribute__( (eds, space(dma)) );
//    __eds__ int bufferB[NUMSAMP] __attribute__( (eds, space(dma)) );
//#else
//    int	bufferA[NUMSAMP] __attribute__( (space(xmemory)) );
//    int	bufferB[NUMSAMP] __attribute__( (space(xmemory)) );
//#endif

void configureADC(void);
int analogRead(void);

int main(void) {
    configureUART();
    configureADC();

    delay_s(3);

    int buffer[1000];

    int i = 0;
    for(i = 0; i < 1000; i++) {
        buffer[i] = analogRead();
    }

    for(i = 0; i < 1000; i++) {
        println(printf("%d,", buffer[i]));
    }

//    while(1) {
//        println(printf("%d", analogRead()));
//        delay_us(1);
//    }
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