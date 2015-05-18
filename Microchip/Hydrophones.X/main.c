#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #include <p33Exxxx.h>
#endif

#include <string.h>
#include <stdlib.h>

#include "config.h"
#include "util.h"
//#include "global.h"

int main(void)
{
    initApp();
   
    int buffer[4000];

    int i;
    for(i = 0; i < 4000; i += 4) {
        buffer[i] = ADC1BUF1;
        buffer[i + 1] = ADC1BUF2;
        buffer[i + 2] = ADC1BUF3;
        buffer[i + 3] = ADC1BUF0;
    }

    for(i = 0; i < 4000; i += 4) {
//        char *tempMsg1;
//        intToString(buffer[i], tempMsg1);
//        char *tempMsg2;
//        intToString(buffer[i + 1], tempMsg2);
//        char *tempMsg3;
//        intToString(buffer[i + 2], tempMsg3);
//        char *tempMsg4;
//        intToString(buffer[i + 3], tempMsg4);
//
//        char *msg;
//        strcat(msg, tempMsg1);
//        strcat(msg, ",");
//        strcat(msg, tempMsg2);
//        strcat(msg, ",");
//        strcat(msg, tempMsg3);
//        strcat(msg, ",");
//        strcat(msg, tempMsg4);
//
//        println(msg);

        println(printf("%d,%d,%d,%d," , buffer[i], buffer[i + 1], buffer[i + 2], buffer[i + 3]));
    }
//
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