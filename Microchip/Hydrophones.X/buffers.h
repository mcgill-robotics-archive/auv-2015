#ifndef BUFFERS
#define	BUFFERS

typedef enum {BURST, PASSIVE} bufType;

void initializeBuffers(void);
void store(int value, int hydroNum, int valNum, bufType buf);
int get(int hydroNum, int valNum, bufType buf);

#endif	//buffers