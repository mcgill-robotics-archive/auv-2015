#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #include <p33Exxxx.h>
#endif

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "config.h"
#include "util.h"
#include "interrupts.h"
#include "buffers.h"


int main(void)
{
    // This is what was used to produce the data as seen on Podio
    // to run this, comment configureInterrupts() and initializeBuffer() in config.c

    initApp();

    int buffer[4][1000];
    int i;

    while(1) { // Note the sample operation is much quicker than the transfer
        for(i = 0; i < 1000; i++) {
            buffer[0][i] = sample(1);
            buffer[1][i] = sample(2);
            buffer[2][i] = sample(3);
            buffer[3][i] = sample(4);
        }

        for(i = 0; i < 1000; i++) {
            println((char*) printf("%d %d %d %d " , \
                    buffer[0][i], \
                    buffer[1][i], \
                    buffer[2][i], \
                    buffer[3][i]));
        }
    }

    // This code is untested, but attempts to use interrupts
    // to run it, comment the code above, and uncomment configureInterrupts()
    // and initializeBuffer() in config.c

    initApp();

    while(1) {
        while(!getBufferStatus());

        setBufferStatus(0);
        transmitSample(get(1, BURST), 1);
        transmitSample(get(2, BURST), 2);
        transmitSample(get(3, BURST), 3);
        transmitSample(get(4, BURST), 4);
    }

    while(1);
    return 0;
}