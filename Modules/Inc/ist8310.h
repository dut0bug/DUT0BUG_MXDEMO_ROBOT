#ifndef _IST8310_H_
#define _IST8310_H_

#include "main.h"

#define IST8310_ADDRESS 0x0E

extern float ist8310_mag[3];

void IST8310_init(void);
void IST8310_read_mag(void);


#endif
