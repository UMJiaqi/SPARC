#include <string.h>
#include <vector>
#include <stdio.h>
#include "common.h"
#include "cavity.h"
#include "MPCR.h"
#include "gen_path.h"
#include "sensor.h"
#include "communicator.h"
#include "pure_pursuit.h"
#include "characterization.h"
#include "keyboard_controller.h"
#include "max_turning_angle.h"

const DTYPE Lh2 = 0.03 * 0.03;  // square of "look head distrance"
const int PATH_TYPE = STRAIGHT_PATH;    // path shape

void path_following(MPCR *MPCR_model) {
    const int control_type = CLOSED_LOOP_CONTROL;//OPEN_LOOP_CONTROL;//CLOSED_LOOP_CONTROL;
    const int mode = 1;

    // Initialize optitrack
    sensor *msensor = nullptr;
    //msensor = new sensor();
    if (control_type == OPEN_LOOP_CONTROL) {
        msensor = nullptr;
    } else {
        msensor = new sensor();
    }

    // Initialize arduino and serial port
    communicator *mcommunicator = new communicator("/dev/ttyACM0", MPCR_model->get_section_num(), mode);

    // Generate reference path
    std::vector<std::vector<DTYPE>> path;
    gen_path(MPCR_model->get_foot_z(0), MPCR_model->get_foot_x(0), MPCR_model->get_max_l(), PATH_TYPE, path);
    
    // Initialize the controller
    PPC *PPC_controller = new PPC(MPCR_model, msensor, mcommunicator, path, Lh2, control_type);

    // Begin to move
    PPC_controller->start();
    
    if (msensor != nullptr) delete msensor;
    if (mcommunicator != nullptr) delete mcommunicator;
    if (PPC_controller != nullptr) delete PPC_controller;
}

void characterzation() {
    const int mode = 2;
    sensor *msensor = new sensor();
    communicator *mcommunicator = new communicator("/dev/ttyACM0", 1, mode);
    characterization *character = new characterization(msensor, mcommunicator);
    character->start();
    if (msensor != nullptr) delete msensor;
    if (mcommunicator != nullptr) delete mcommunicator;
    if (character != nullptr) delete character;
}

void simulation(MPCR *MPCR_model) {
    const int control_type = OPEN_LOOP_CONTROL;
    const int mode = 3;

    // Initialize optitrack
    sensor *msensor = nullptr;

    // Initialize arduino and serial port
    communicator *mcommunicator = nullptr;

    // Generate reference path
    std::vector<std::vector<DTYPE>> path;
    gen_path(MPCR_model->get_foot_z(0), MPCR_model->get_foot_x(0), MPCR_model->get_max_l(), PATH_TYPE, path);

    // Initialize the controller
    PPC *PPC_controller = new PPC(MPCR_model, msensor, mcommunicator, path, Lh2, control_type);

    // Begin to move
    PPC_controller->start(mode);

    if (msensor != nullptr) delete msensor;
    if (mcommunicator != nullptr) delete mcommunicator;
    if (PPC_controller != nullptr) delete PPC_controller;
}

void keyboard(MPCR *MPCR_model) {
    communicator *mcommunicator = new communicator("/dev/ttyACM0", MPCR_model->get_section_num(), 1);

    keyboard_controller *keyctrl = new keyboard_controller(MPCR_model, mcommunicator);

    keyctrl->start();

    if (mcommunicator != nullptr) delete mcommunicator;
}

void get_max_turning_angle(MPCR *MPCR_model) {
    communicator *mcommunicator = new communicator("/dev/ttyACM0", MPCR_model->get_section_num(), 1);

    max_turning_angle(MPCR_model, mcommunicator);

    if (mcommunicator != nullptr) delete mcommunicator;
}

int main() {
    // The parameters of the cavity
    DTYPE min_l = 0.070;           // m
    DTYPE max_l = 0.095;           // m
    int n = 6;
    DTYPE a = 14;                 // mm
    DTYPE A0 = 9.6018;              // mm
    DTYPE B0 = 0.0160;              // angle
    DTYPE C0 = 7.9928;              // mm
    // The parameters of the MPCR
    int n_section = 1;
    DTYPE r = 23.09 * 1e-3;            // m
    DTYPE coord0[2]{0, 0};          // m
    DTYPE theta0 = 0 * PI / 180;    // radian
    // mode: 1 for path following, 2 for characterzation, 3 for simulation, 4 for keyboard controller, 5 for max turning angle
    int mode = 1;

    // Initialize the cavity
    cavity *mcavity = new cavity(n, min_l, max_l, a, A0, B0, C0);

    // Initialize the mobile PCR
    MPCR *MPCR_model = new MPCR(mcavity, n_section, r, coord0, theta0);
    
    if (mode == 1) {
        path_following(MPCR_model);
    } else if (mode == 2) {
        characterzation();
    } else if (mode == 3) {
        simulation(MPCR_model);
    } else if (mode == 4) {
        keyboard(MPCR_model);
    } else if (mode == 5) {
        get_max_turning_angle(MPCR_model);
    }

    // Free memory
    if (mcavity != nullptr) delete mcavity;
    if (MPCR_model != nullptr) delete MPCR_model;
}
