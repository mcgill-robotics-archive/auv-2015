#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    	#include <p33Exxxx.h>
#endif

#include <stdint.h>
#include <stdbool.h>
#include <libpic30.h>

#define CRYSTAL 2800000UL
#define FCY CRYSTAL*50UL/8UL

#define delay_us(d) { __delay32( (unsigned long) ((d)*(FCY)/1000000UL)); }
#define delay_ns(d) { __delay32( (unsigned long) ((d)*(FCY)/1000000000UL)); }

#pragma config JTAGEN = OFF     //should not matter. only affects pins TDO, TCK, TDI,TMS
#pragma config FWDTEN = OFF     //watchdog timer. should not be relevant

// Select Internal FRC at POR
_FOSCSEL(FNOSC_FRC & IESO_OFF);
// Enable Clock Switching and Configure Primary Oscillator in XT mode
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);

void configureClock(void);

int main() {
    configureClock();
    
    TRISDbits.TRISD1 = 0;       // set output
    LATDbits.LATD1 = 1;         // set high

    while(1) {
        LATDbits.LATD1 = !PORTDbits.RD1;
        delay_ns(100);
    }
    return 1;
}

void configureClock(void) {
    /*
     * This sets FCY  to 46875000
     */
    PLLFBDbits.PLLDIV = 48;         // M = 50
    CLKDIVbits.PLLPRE = 0;          // N2 = 2
    CLKDIVbits.PLLPOST = 0;         // N1 = 2
    // Initiate Clock Switch to FRC oscillator with PLL (NOSC=0b001)
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(OSCCON | 0x01);
    // Wait for Clock switch to occur
    while (OSCCONbits.COSC!= 0b001);
    // Wait for PLL to lock
    while(OSCCONbits.LOCK != 1);
}