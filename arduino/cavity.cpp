#include "cavity.h"
#include "AS5600_IIC.h"
#include "DAC.h"

int init_angle_4096[2][3]{0};           // Initial encoder reading (absolute value).
int pre_angle_4096[2][3]{0};            // Previous encoder reading (relative to initial value).
int n_turns[2][3] = {0};                // Current encoder revolutions.
double voltage[2][3]{0};                // Current proportional valve voltage.

// PID
double integral_bias[2][3]{0};          // Cumulative error in PID control.
double last_input[2][3]{0};
unsigned long last_time[2][3]{0};

const unsigned int sample_time = 40;  //40ms
/*const double Kp = 0.004;
const double Ki = 0.00000;
const double Kd = 0.08;*/
/*const double Kp = 0.004;
const double Ki = 0.000;
const double Kd = 0.06;*/
const double Kp = 0.004;
const double Ki = 0.000;
const double Kd = 0.06;

template<typename T>
T max3(const T x1, const T x2, const T x3) {
    T res = x1;
    res = res > x2? res : x2;
    res = res > x3? res : x3;
    return res;
}

void init_cavities(int n_section) {
    // Initialize encoder, DAC.
    for (int i = 0; i < n_section; ++i) {
        for (int j = 0; j < 3; ++j) {
            pinMode(encoder_pins[i][j][0], OUTPUT);
            pinMode(encoder_pins[i][j][1], OUTPUT);
            digitalWrite(encoder_pins[i][j][0], 1);
            digitalWrite(encoder_pins[i][j][0], 1);
            init_DAC_out_port(i * 3 + j);
            write_DAC(i * 3 + j, 0);
        }
    }
    delay(2000);
    // Initial deflection angle.
    for (int i = 0; i < n_section; ++i) {
        for (int j = 0; j < 3; ++j) {
            init_angle_4096[i][j] = read_sensor_values(encoder_pins[i][j][0], encoder_pins[i][j][1]);
        }
    }
}

// Read encoder angle.
double get_angle(const int section, const int cavity) {
    int data = read_sensor_values(encoder_pins[section][cavity][0], 
                encoder_pins[section][cavity][1]) - init_angle_4096[section][cavity];
    int res;
    if (data < 0) {
      data = data + 4096;
    }
    if (data - pre_angle_4096[section][cavity] > 3500) { // A sudden increase indicates reverse rotation crossing over the zero point.
      n_turns[section][cavity]--;
    } else if (data - pre_angle_4096[section][cavity] < -3500) { // A sudden decrease indicates forward rotation crossing over the zero point.
      n_turns[section][cavity]++;
    }
    res = data + n_turns[section][cavity] * 4096;

    pre_angle_4096[section][cavity] = data;

    return -(double)res * 360 / 4096;
}

// Position closed-loop.
double Position_PID(const int section, const int cavity, const double input, const double target) {
    unsigned long now = millis();
    unsigned long time_change = (now - last_time[section][cavity]);
    //Serial.println(time_change);
    if (time_change >= sample_time) {
      double error = target - input;    // Error
      integral_bias[section][cavity] += error; // Cumulative error
      double derivative = input - last_input[section][cavity]; // Derivative

      // Calculate PID output.
      double out = Kp * error + Ki * integral_bias[section][cavity] - Kd * derivative;

      // Update parameters.
      last_input[section][cavity] = input;
      last_time[section][cavity] = now;
      return out;
    }
    return -10000;
}

void head_up(const double _target_angle[2][3], const int mask[2][3]) {
    if (_target_angle[0][0] > 150 && _target_angle[0][1] > 150 && _target_angle[0][2] > 150) return;  //No need to raise the head during maximum contraction.

    double target_angle[2][3]{{80, 40, 40},{80, 40, 40}};
    //double target_angle[2][3]{{120, 40, 40},{120, 40, 40}};

    // PID Move to the raised head position.
    double cur_angle[2][3]{0}, error = -10000;
    // clear PID integral bias and error
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 3; ++j) {
          integral_bias[i][j] = 0;
      }
    }
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 3; ++j) {
            cur_angle[i][j] = get_angle(i, j);
            if (mask[i][j] != 0) {
                error = max(abs(target_angle[i][j] - cur_angle[i][j]), error);
            }
        }
    }
  
    int count = 0;
    int loop_i = 0;
    while (count < 1) {
        error = -10000;
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (mask[i][j] != 0) {
                    double update = Position_PID(i, j, cur_angle[i][j], target_angle[i][j]);
                    if (update >= -10) {
                        voltage[i][j] += update;
                        if (voltage[i][j] > 10) voltage[i][j] = 10;
                        if (voltage[i][j] < 0) voltage[i][j] = 0;
                        write_DAC(i * 3 + j, voltage[i][j]);  // 10ms latency
                    }
                }
            }
        }
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (mask[i][j] != 0) {
                    cur_angle[i][j] = get_angle(i, j);      // 1ms latency
                    error = max(abs(target_angle[i][j] - cur_angle[i][j]), error);
                }
            }
        }
        if (error < 10) {
          count += 1;
        } else {
          count = 0;
        }
    }
}

void mask_PID_controll_cavity(const double target_angle[2][3], const int mask[2][3]) {
    //head_up(target_angle, mask);

    double cur_angle[2][3]{0}, error = -10000;

    // clear PID integral bias and error
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 3; ++j) {
            integral_bias[i][j] = 0;
        }
    }

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 3; ++j) {
            cur_angle[i][j] = get_angle(i, j);
            if (mask[i][j] != 0) {
                error = max(abs(target_angle[i][j] - cur_angle[i][j]), error);
            }
        }
    }
  
    int count = 0;
    int loop_i = 0;
    // 30, 10
    // 50, 3
    while (count < 50) {
        error = -10000;
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (mask[i][j] != 0) {
                    double update = Position_PID(i, j, cur_angle[i][j], target_angle[i][j]);
                    if (update >= -10) {
                        voltage[i][j] += update;
                        if (voltage[i][j] > 10) voltage[i][j] = 10;
                        if (voltage[i][j] < 0) voltage[i][j] = 0;
                       // Serial.print("voltage:");
                       // Serial.println(voltage[i][j]);
                        write_DAC(i * 3 + j, voltage[i][j]);  // 10ms latency
                    }
                }
            }
        }
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 3; ++j) {
                if (mask[i][j] != 0) {
                    cur_angle[i][j] = get_angle(i, j);      // 1ms latency
                    error = max(abs(target_angle[i][j] - cur_angle[i][j]), error);
                }
            }
        }
        if (error < 4) {
          count += 1;
        } else {
          count = 0;
        }
        loop_i ++;
    }
}


// status:1 -> max negative pressure
// status:0 -> min negative pressure
void mask_OPEN_control_cavity(const double target_angle[2][3], const int mask[2][3], const int status) {
    double cur_angle[2][3]{0}, error = 10000;
    // control cavity
    pinMode(40, OUTPUT);
    digitalWrite(40, status);

    // moniter angle
    while (error > 50) {
        error = -1;
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 3; ++j) {
                cur_angle[i][j] = get_angle(i, j);      // 1ms latency
                if (mask[i][j] != 0) {
                    error = max(abs(target_angle[i][j] - cur_angle[i][j]), error);
                }
            }
        }
        Serial.print("angle:");
        Serial.println(cur_angle[0][0]);
    }
}

void print_angle() {
    String msg = "";
    double angle[6];
    angle[0] = get_angle(0, 0);
    angle[1] = get_angle(0, 1);
    angle[2] = get_angle(0, 2);
    angle[3] = get_angle(1, 0);
    angle[4] = get_angle(1, 1);
    angle[5] = get_angle(1, 2);
    msg += String(angle[0]);
    msg += String(",");
    msg += String(angle[1]);
    msg += String(",");
    msg += String(angle[2]);
    msg += String(",");
    msg += String(angle[3]);
    msg += String(",");
    msg += String(angle[4]);
    msg += String(",");
    msg += String(angle[5]);
    msg += String("\n");
    Serial.print(msg);
}
