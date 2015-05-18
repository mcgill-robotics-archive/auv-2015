#include <libpic30.h>

#include "config.h"

#ifndef delay_s(d)
    #define delay_s(d) { __delay32( (unsigned long) (((unsigned long long) d)*(FCY))); }
    #define delay_ms(d) { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000ULL)); }
    #define delay_us(d) { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000000ULL)); }
    #define delay_ns(d) { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000000000ULL)); }
#endif

void println(const char *line);
void transmitSample(const int *hydroSample, int hydroNum);
int sample(int hydrophone);