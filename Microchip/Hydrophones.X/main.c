#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #include <p33Exxxx.h>
#endif

#include "config.h"
#include "util.h"
//#include "global.h"

int main(void)
{
    initApp();

    while(1) {
        println(printf("%d,%d,%d,%d,", ADC1BUF1, ADC1BUF2, ADC1BUF3, ADC1BUF0));
        delay_ms(100);
    }

//    int burstBuffer[4][1000];
//    //int passiveBuffer[4][1000];
//
//    int i = 0;
//    for(i = 0; i < 1000; i++) {
//        burstBuffer[0][i] = sample(1);
//        burstBuffer[1][i] = sample(2);
//        burstBuffer[2][i] = sample(3);
//        burstBuffer[3][i] = sample(4);
//    }
//
//    for(i = 0; i < 1000; i++) {
//        println(printf("%d,%d,%d,%d,", burstBuffer[0][i], burstBuffer[1][i], burstBuffer[2][i], burstBuffer[3][i]));
//    }
//
//    for(i = 0; i < 4; i++) {
//        transmitSample(burstBuffer[i], i + 1);
//    }

    while(1);
    return 0;
}