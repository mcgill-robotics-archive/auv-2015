#ifndef FCY
    #define INTERNAL_OSCILLATOR 7370000
    #define M 100           // subject to power restrictions
    #define N1 2            // 2 is minimum
    #define N2 2            // 2 is minimum
     /*
      * FCY = ((input clock) * M/(N1 * N2)/2
      * Equation 9-1 and 9-2 in datasheet
      */
    #define FCY INTERNAL_OSCILLATOR * M/(N1 * N2 * 2)
#endif

void initApp(void);