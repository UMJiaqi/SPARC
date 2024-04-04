#include "keyboard_controller.h"
#include <iostream>

/*  Control Keys: 
* w: up, a: left, s: down, d: right, j: back, k: front 
* u: change chambers 
* n: decrease 1mm dl, m: increase 1mm dl
 */
keyboard_controller::keyboard_controller(
    MPCR *mMPCR,
    communicator *mcommunicator)
{
    keyctrl_MPCR = mMPCR;
    keyctrl_communicator = mcommunicator;
    dvol = 0.01; //5mm
    cur_voltage.push_back(0);
    cur_voltage.push_back(0);
    cur_suckers.push_back(0);
    cur_suckers.push_back(0);
    cur_suckers.push_back(0);
}

void keyboard_controller::start() {
    std::cout << "[LOG INFO] start keyboard_controller" << std::endl;


/*    if (keyctrl_communicator != nullptr) {
        auto l_min_angle = keyctrl_MPCR->convert_l_to_angle(keyctrl_MPCR->get_min_l());
        // 0 Foot Fixed, 1 Foot Released
        keyctrl_communicator->control_sucker('f', 'r');
        // Chamber 0-1 contracts to its maximum.
        keyctrl_communicator->control_cavity(l_min_angle, l_min_angle, l_min_angle);
        // 0 release, 1 fix
        keyctrl_communicator->control_sucker('r', 'f');
    }
*/

    int argc = 0;
    ros::init(argc, nullptr, "controller_keyboard");
    ros::NodeHandle nh;
    ros::Subscriber ros_tutorial_sub = nh.subscribe("keyboard", 10, &keyboard_controller::callback, this);
    ros::spin();
}

bool check_valid(const std::string &c) {
    if (c[0] == 'w' || c[0] == 's' || c[0] == 'i' || c[0] == 'k' || c[0] == 'n' || c[0] == 'm') return true;
    return false;
}

void keyboard_controller::control_vol(char c) {
    std::vector<DTYPE> target_vol;
    target_vol.push_back(cur_voltage[0]);
    target_vol.push_back(cur_voltage[1]);
    if (c == 'w') {
        target_vol[0] += dvol;      // y += dl
    } else if (c == 's') {
        target_vol[0] -= dvol;      // x += dl
    } else if (c == 'i') {
        target_vol[1] += dvol;      // y -= dl
    } else if (c == 'k') {
        target_vol[1] -= dvol;      // x -= dl
    }
    if (target_vol[0] >= 0 && target_vol[0] <= 10 && target_vol[1] >= 0 && target_vol[1] <= 10) {
        cur_voltage[0] = target_vol[0];
        cur_voltage[1] = target_vol[1];
        
        std::cout << "[LOG INFO] vol0:" << cur_voltage[0] << ", vol1:" << cur_voltage[1] << std::endl;
    } else {
        std::cout << "[LOG INFO] out of scale" << std::endl;
    }
}

void keyboard_controller::change_dvol(char c) {
    if (c == 'n') {
        dvol -= 0.01;
    } else if (c == 'm') {
        dvol += 0.01;
    }
    std::cout << "[LOG INFO] change dvol to " << dvol << std::endl;
}

void keyboard_controller::control_sucker(char c) {
    if (c == 'z') {
        cur_suckers[0] != cur_suckers[0];
    } else if (c == 'x') {
        cur_suckers[1] != cur_suckers[1];
    } else if (c == 'c') {
        cur_suckers[2] != cur_suckers[2];
    }
    char status[3];
    status[0] = cur_suckers[0] == 0 ? 'r' : 'f';
    status[1] = cur_suckers[1] == 0 ? 'r' : 'f';
    status[2] = cur_suckers[2] == 0 ? 'r' : 'f';
    keyctrl_communicator->control_sucker(status[0], status[1], status[2]);
}

void keyboard_controller::callback(const std_msgs::String::ConstPtr& msg) {
    if (check_valid(msg->data)) {
        char c =  msg->data[0];
        if (c == 'w' || c == 's' || c == 'i' || c == 'k') control_vol(c);
        if (c == 'z' || c == 'x' || c == 'c') control_sucker(c);
        if (c == 'n' || c == 'm') change_dvol(c);
    }
}
