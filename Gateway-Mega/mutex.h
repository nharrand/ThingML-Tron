#ifndef mutex_h
#define mutex_h

#include <Arduino.h>
#include "mutex.c"

void mutex_setup();
bool orn(bool * tab, uint8_t p);
void mutex_lock(uint8_t r, uint8_t p);
void mutex_unlock(uint8_t r, uint8_t p);

#endif
