#ifndef DAC
#define DAC

#include "LTC2664.h"
#include <stdint.h>
#include <SPI.h>

#define OUT_PORT0 0
#define OUT_PORT1 1
#define OUT_PORT2 2
#define OUT_PORT3 3

void init_DAC_out_port(int out_port);
void write_DAC(const int out_port, const float voltage);

#endif