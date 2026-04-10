#if !defined(VACUUM_H)
#define VACUUM_H

#include <Arduino.h>

void vacuum_init();

void vacuum_set(uint16_t duty);
uint16_t vacuum_get(void);

#endif // VACUUM_H
