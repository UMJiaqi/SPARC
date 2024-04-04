#include "DAC.h"
#include <Arduino.h>

int init_pin_flag = false;

void init_DAC_out_port(const int out_port) {
    if (!init_pin_flag) {
        pinMode(SS, OUTPUT);
        digitalWrite(SS, HIGH);
        SPI.begin();
        SPI.setClockDivider(SPI_CLOCK_DIV16);
        init_pin_flag = true;
    }
    uint16_t dac_code = LTC2664_voltage_to_code(0, 0, 10);
    LTC2664_write(SS, LTC2664_CMD_SPAN, out_port, LTC2664_SPAN_0_TO_10V);
    delay(10);
    LTC2664_write(SS, LTC2664_CMD_WRITE_N_UPDATE_N, out_port, dac_code);
    delay(10);
}

void write_DAC(const int out_port, const float voltage) {
    uint16_t dac_code = LTC2664_voltage_to_code(voltage, 0, 10);
    LTC2664_write(SS, LTC2664_CMD_SPAN, out_port, LTC2664_SPAN_0_TO_10V);
    delay(10);
    uint8_t ret = LTC2664_write(SS, LTC2664_CMD_WRITE_N_UPDATE_N, out_port, dac_code);
}
