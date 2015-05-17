#ifndef FCY
    #define FCY 46875000UL

    #define delay_ns(d) { __delay32( (unsigned long) ((d)*(FCY)/1000000000UL)); }
    #define delay_us(d) { __delay32( (unsigned long) ((d)*(FCY)/1000000UL)); }
    #define delay_ms(d) { __delay32( (unsigned long) ((d)*(FCY)/1000UL)); }
    #define delay_s(d)  { __delay32( (unsigned long) ((d)*(FCY)/1UL)); }
#endif

void println(const char *line);
void configureUART(void);
void configureClock(void);