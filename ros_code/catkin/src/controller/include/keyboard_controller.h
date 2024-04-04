#ifndef KERBOARD_CONTROLLER
#define KERBOARD_CONTROLLER

#include "common.h"
#include "MPCR.h"
#include "communicator.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

class keyboard_controller {
    private:
        DTYPE dvol;                                         // distance per command
        std::vector<DTYPE> cur_voltage;                     // current voltage
        std::vector<DTYPE> cur_suckers;                     // current status of suckers
        MPCR *keyctrl_MPCR;                                 // pointer of MPCR model
        communicator *keyctrl_communicator;                 // pointer of arduino communicator
        void callback(const std_msgs::String::ConstPtr& msg);
        void control_vol(char c);
        void control_sucker(char c);
        void change_dvol(char c);
    public:
        keyboard_controller(MPCR *, communicator *);
        void start();
};

#endif
