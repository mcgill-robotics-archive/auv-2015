#include <libpic30.h>

#include "config.h"

#ifndef delay_ns(d)
    #define delay_ns(d) { __delay32( (unsigned long) ((d)*(FCY)/1000000000UL)); }
    #define delay_us(d) { __delay32( (unsigned long) ((d)*(FCY)/1000000UL)); }
    #define delay_ms(d) { __delay32( (unsigned long) ((d)*(FCY)/1000UL)); }
    #define delay_s(d)  { __delay32( (unsigned long) ((d)*(FCY)/1UL)); }
#endif

void println(const char *line);
void transmitSample(const int *hydroSample, int hydroNum);
int sample(int hydrophone);