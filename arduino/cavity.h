#ifndef CAVITY
#define CAVITY

#include <Arduino.h>

#define CAVITY1 0
#define CAVITY2 1
#define CAVITY3 2

extern const int encoder_pins[2][3][2];
extern double voltage[2][3];                // Current voltage for proportional valve.
void init_cavities(int);

void mask_PID_controll_cavity(const double target_angle[2][3], const int mask[2][3]);

double get_angle(const int section, const int cavity);

void mask_OPEN_control_cavity(const double target_angle[2][3], const int mask[2][3], const int status);

void print_angle();

#endif
