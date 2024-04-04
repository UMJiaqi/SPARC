#include <Arduino.h>         // Arduino Library
#include <stdint.h>          // Definition of Data Types

#include "common.h"
#include "DAC.h"
#include "cavity.h"         // Control Cavity
#include "sucker.h"         // Control Sucker

/* 
 * Initialization Command Encoding:
 *   x,y
 *   x: The number of serial SPARC currently only support 1, 2, 3, 4, 5.
 *   y: Motion modes, 1- Path tracking, 2- Calibration, 4- Open-loop linear pulse, 5- Double-joint open loop.
 *  
 * Runtime instruction encoding:  
* Control Sucker: S, f, r, f S represents sucker, the following f, r respectively represent fix and release  
* Control Cavity: C, x, x, x, y, y, y, C represents cavity, the following x, x, x represents the cavity angle of PCR 0, y, y, y represents the cavity angle of PCR 1    
* Pin definitions:  
* The six cavity encoder pins corresponding to the six cavities of the two PCRs: {{{2,3}, {4,5}, {6,7}}, {{8,9}, {10,11}, {12,13}}} in order {SCK, SDA}  
* 3 sucker pins {22, 23, 24} {negative pressure}  
* DAC pins {50, 51, 52, 53}  
* Pins of the linear open-loop motion control cavity solenoid valve {40}
 */

#define PATH_FOLLOWING 1

const int sucker_pins[3] = {22, 23, 24};
const int encoder_pins[2][3][2] = {{{2, 3}, {4, 5}, {6, 7}}, {{8, 9}, {10, 11}, {12, 13}}};
int n_section;
int mode;

void setup() {
    Serial.begin(9600);
    init_suckers();
    init_cavities(2);
    while (true) {
        if (Serial.available() > 0) { // Waiting for the host computer to send configuration.
            String cmd = Serial.readString();
            n_section = String(cmd[0]).toInt();
            cmd = cmd.substring(cmd.indexOf(',') + 1, cmd.length());
            mode = cmd.toInt();
            break;
        }
    }

    init_cavities(n_section);
    init_suckers();

    Serial.print("ok");
    delay(1000);
}

void loop() {
    if (mode == 1) {
        path_following();
    }
    if (mode == 2) {
        characterization();
    }
    if (mode == 4) {
        open_straight();
    }
    if (mode == 5) {
        open_double_section();
    }
}

void path_following() {
    if (Serial.available() > 0) {
        String cmd = Serial.readString();
        if (cmd[0] == 'C') {
            double target_angle[2][3]{0};
            int mask[2][3]{0};
            // example "C,123.4,41.2,2.31,-1,-1,-1", -1:Stationary	
            cmd = cmd.substring(cmd.indexOf(',') + 1, cmd.length());
            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 3; ++j) {
                    String str_angle = cmd.substring(0, cmd.indexOf(','));
                    cmd = cmd.substring(cmd.indexOf(',') + 1, cmd.length());
                    target_angle[i][j] = str_angle.toFloat();
                    if (target_angle[i][j] >= 0) {
                        mask[i][j] = 1;
                    }
                }
            }
            mask_PID_controll_cavity(target_angle, mask);
            Serial.print("ok");
        } else if (cmd[0] == 'S') {
            // example "S,f,r,f"
            int status[3]{0};
            for (int i = 0; i < n_section + 1; ++i) {
                status[i] = (cmd[i * 2 + 2] == 'f' ? 1 : 0);
            }
            control_suckers(status);
            Serial.print("ok");
        }
    }
}

void characterization() {
    if (Serial.available() > 0) {
        String cmd = Serial.readString();
        if (cmd[0] == 'C') {
            double target_angle[2][3]{0};
            int mask[2][3]{0};
            // example "C,123.4,41.2,2.31,-1,-1,-1", -1:Stationary
            cmd = cmd.substring(cmd.indexOf(',') + 1, cmd.length());
            String str_angle = cmd.substring(0, cmd.indexOf(','));
            cmd = cmd.substring(cmd.indexOf(',') + 1, cmd.length());
            target_angle[0][0] = str_angle.toFloat();
            mask[0][0] = 1;
            mask_PID_controll_cavity(target_angle, mask);
            Serial.print("ok");
        }
    }
}

void open_straight() {
    if (Serial.available() > 0) {
        String cmd = Serial.readString();
        int num_step = cmd.toInt();
        if (n_section == 1) {
            for (int step = 0; step < num_step; ++step) {
                int sucker_status[3]{0};
                double target_angle[2][3]{0};
                int mask[2][3]{0};
                // Fore foot fixed
                sucker_status[0] = 1;
                sucker_status[1] = 0;
                sucker_status[2] = 0;
                control_suckers(sucker_status);
                // Cavity contracts to the shortest
                for (int i = 0; i < 3; ++i) {
                    target_angle[0][i] = 290;
                    mask[0][i] = 1;
                }
                mask_OPEN_control_cavity(target_angle, mask, 1);
                Serial.print("end cavity");
                // Rear foot fixed.
                sucker_status[0] = 0;
                sucker_status[1] = 1;
                sucker_status[2] = 0;
                control_suckers(sucker_status);
                // The cavity is released to the longest.
                for (int i = 0; i < 3; ++i) {
                    target_angle[0][i] = 10;
                    mask[0][i] = 1;
                }
                mask_OPEN_control_cavity(target_angle, mask, 0);
                // Release the rear foot.
                sucker_status[0] = 0;
                sucker_status[1] = 0;
                sucker_status[2] = 0;
                control_suckers(sucker_status);                
            }
        }
        if (n_section == 2) {
            // pass
        }
    }
}

void open_double_section() {
    if (Serial.available() > 0) {
        String cmd = Serial.readString();
        if (cmd[0] == 'V') {
            // example "V,1,0,0,1,0,0"
            cmd = cmd.substring(cmd.indexOf(',') + 1, cmd.length());
            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 3; ++j) {
                    String str_vol = cmd.substring(0, cmd.indexOf(','));
                    cmd = cmd.substring(cmd.indexOf(',') + 1, cmd.length());
                    write_DAC(i * 3 + j, str_vol.toFloat());
                }
            }
            //Serial.print("ok");
        }
        if (cmd[0] == 'S') {
            // example "S,f,r,f"
            int status[3]{0};
            for (int i = 0; i < n_section + 1; ++i) {
                status[i] = (cmd[i * 2 + 2] == 'f' ? 1 : 0);
            }
            control_suckers(status);
            //Serial.print("ok");
        }
    } /*else {
          // Output encoder angle.
          //print_angle();
          String msg = "";
          double vol[6];
          vol[0] = analogRead(A8) * (5.0 / 1023.0);
          vol[1] = analogRead(A9) * (5.0 / 1023.0);
          vol[2] = analogRead(A10) * (5.0 / 1023.0);
          vol[3] = analogRead(A11) * (5.0 / 1023.0);
          vol[4] = analogRead(A12) * (5.0 / 1023.0);
          vol[5] = analogRead(A13) * (5.0 / 1023.0);
          msg += String(vol[0], 4);
          msg += String(",");
          msg += String(vol[1], 4);
          msg += String(",");
          msg += String(vol[2], 4);
          msg += String(",");
          msg += String(vol[3], 4);
          msg += String(",");
          msg += String(vol[4], 4);
          msg += String(",");
          msg += String(vol[5], 4);
          //Serial.print(msg);
          delay(50);
    }*/
}
