#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #include <p33Exxxx.h>
#endif

#include <stdlib.h>

#include "buffers.h"

int **burstBuffer;
int **passiveBuffer;

void initializeBuffers(void) {
    burstBuffer = malloc(sizeof(*burstBuffer) * 4);
    passiveBuffer = malloc(sizeof(*passiveBuffer) * 4);

    int i;
    for (i = 0; i < 4; i++) {
        burstBuffer[i] = malloc(sizeof(int) * 1000);
        passiveBuffer[i] = malloc(sizeof(int) * 1000);
    }
}

void store(int value, int hydroNum, int valNum, bufType buf) {
    switch(buf) {
    case BURST:
        burstBuffer[hydroNum - 1][valNum] = value;
        break;
    case PASSIVE:
        passiveBuffer[hydroNum - 1][valNum] = value;
        break;
    }
}

int get(int hydroNum, int valNum, bufType buf) {
    switch(buf) {
    case BURST:
        return burstBuffer[hydroNum - 1][valNum];
    case PASSIVE:
        return passiveBuffer[hydroNum - 1][valNum];
    default:
        return 0;
    }
}