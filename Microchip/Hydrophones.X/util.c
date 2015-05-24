#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #include <p33Exxxx.h>
#endif

#include <string.h>

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

void transmitSample(const int *hydroSample, int hydroNum) {
    //U1TXREG = begin_flag;
    unsigned int size = sizeof(hydroSample)/sizeof(hydroSample[0]);
    while(size) {
        while(U1STAbits.UTXBF);     // wait while TX buffer full
        U1TXREG = *hydroSample;

        hydroSample++;
        size--;
    }
    while(U1STAbits.UTXBF);         // wait while TX buffer full
    //U1TXREG = end_flag;
    while(!U1STAbits.TRMT);         // wait for trasmit to finish
}

int sample(int hydroNum) {
    switch(hydroNum) {
    case 1:
        return ADC1BUF1;
    case 2:
        return ADC1BUF2;
    case 3:
        return ADC1BUF3;
    case 4:
        return ADC1BUF0;
    default:
        return 0;
    }
}