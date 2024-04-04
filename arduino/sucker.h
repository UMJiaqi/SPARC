#ifndef SUCKER
#define SUCKER

#include <Arduino.h>
#include "cavity.h"
#include "DAC.h"

extern const int sucker_pins[3];
void init_suckers();
void control_suckers(const int status[3]);

#endif
