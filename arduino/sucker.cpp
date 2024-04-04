#include "sucker.h"

#define fix(pin)                  \
    digitalWrite(pin,LOW);

#define free(pin)                 \
    digitalWrite(pin,HIGH);

#define open(pin) digitalWrite(pin,HIGH);
#define close(pin) digitalWrite(pin,LOW);

void init_suckers() {
    for(int i = 0; i < 3; ++i) {
      pinMode(sucker_pins[i], OUTPUT);
      free(sucker_pins[i]);
    }
}

void control_suckers(const int status[3]) {

    for (int foot = 0; foot < 3; ++foot) {
        if (status[foot] == 1) {
            //double vol = voltage[0][0] - 0.2;
            //if (vol < 0) vol = 0;
            //write_DAC(0, vol);  // 10ms latency
            fix(sucker_pins[foot]);
        }
    }
    delay(200);

    for (int foot = 0; foot < 3; ++foot) {
        if (status[foot] == 0) {
            free(sucker_pins[foot]);
        }
    }
    //write_DAC(0, voltage[0][0]);  // 10ms latency
}
