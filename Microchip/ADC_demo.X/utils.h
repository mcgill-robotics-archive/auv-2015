#ifndef FCY
    #define INTERNAL_OSCILLATOR 7370000
    #define FCY INTERNAL_OSCILLATOR * 100/8

    #define delay_s(d) { __delay32( (unsigned long) (((unsigned long long) d)*(FCY))); }
    #define delay_ms(d) { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000ULL)); }
    #define delay_us(d) { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000000ULL)); }
    #define delay_ns(d) { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000000000ULL)); }
#endif

void println(const char *line);
void configureUART(void);
void configureClock(void);