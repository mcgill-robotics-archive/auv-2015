#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #include <p33Exxxx.h>
#endif

#include "config.h"
#include "util.h"
//#include "global.h"

extern int **burstBuffer[4];
extern int **passiveBuffer[4];

int main(void)
{
    initApp();
    
    while(1) {
        println("hello");
        delay_s(1);
    }
//    int i = 0;
//    for(i = 0; i < 1000; i++) {
//        burstBuffer[0][i] = sample(1);
//        burstBuffer[1][i] = sample(2);
//        burstBuffer[2][i] = sample(3);
//        burstBuffer[3][i] = sample(4);
//    }
//
//    for(i = 0; i < 4; i++) {
//        transmitSample(burstBuffer[i], i + 1);
//    }

    while(1);
    return 0;
}