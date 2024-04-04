#ifndef COMMUNICATOR
#define COMMUNICATOR

#include <serial/serial.h>
#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include "common.h"

#define TIME_TO_SLEEP 100
#define FORE_FOOT 0
#define HIND_FOOT 1
#define FIX 0
#define UNFIX 1

class communicator {
    private:
        std::string comm_port;
        serial::Serial comm_serial;
    public:
        communicator(const char *port, const int n_section, const int mode);
        ~communicator() {};
        void blocking_send(const std::string &msg);
        void nonblocking_receive(std::string &msg);
        void control_cavity(const DTYPE, const DTYPE, const DTYPE);
        void control_cavity(const DTYPE, const DTYPE, const DTYPE, const DTYPE, const DTYPE, const DTYPE);
        void control_sucker(const char, const char);
        void control_sucker(const char, const char, const char);
        void config(const int n_section, const int mode);
};

inline communicator::communicator(const char *port, const int n_section, const int mode) {
    printf("[LOG INFO] Initialize the communciator\n");
    comm_port = port;
    try {
        comm_serial.setPort(comm_port.c_str());                                 //Set port.
        comm_serial.setBaudrate(9600);                                          //Set baud rate.
        serial::Timeout tout = serial::Timeout::simpleTimeout(TIME_TO_SLEEP);   //Set delay wait in milliseconds.
        comm_serial.setTimeout(tout);
        comm_serial.open();                                                     // Open serial port.
    }
    catch (serial::IOException& e) {
        printf("[error] can't open serial port %s\n", comm_port.c_str());
        exit(-1);
    }

    if (comm_serial.isOpen()) {
        printf("[LOG INFO] serial port %s is open\n", comm_port.c_str());
    } else {
        exit(-1);
    }

    usleep(1000 * 1000);    // Wait for the successful connection of serial communication (1 second).
    
    std::string command;
    command = std::to_string(n_section) + ',' + std::to_string(mode);
    blocking_send(command);
}

/**
 * Send commands to the lower machine, and after the lower machine executes the command, it will return 'ok'.
 * @return void 
 */
inline void communicator::blocking_send(const std::string &msg) {
    printf("[LOG INFO] sending command:%s\n", msg.c_str());
    // Send instructions
    comm_serial.write(msg);

    // Blocking wait for the lower machine to execute the command and return 'ok'.
    std::string rec_msg;
    while (rec_msg != "ok") {
        nonblocking_receive(rec_msg);
        usleep(TIME_TO_SLEEP * 1000);
    }
}

/**
 * Non-blocking receive messages from the lower machine.
 * @param msg Received message.
 * @return void 
 */
inline void communicator::nonblocking_receive(std::string &msg) {
    if (comm_serial.available() > 0) {
        msg = comm_serial.read(comm_serial.available());
    }
}

/* 
 * Command code for initialization:
 *   x,y
 *   x: Number of serially connected PCRs, currently only supports 1,2
 *   y: Motion mode, 1 - path tracking
 *  
 * Command code at runtime:
 *   Control sucker: S,f,r,f S represents sucker, and f,r following represent fix and release respectively
 *   Control cavity: C,x,x,x,y,y,y C represents cavity, x,x,x following are angles of cavity for PCR 0, and y,y,y represent angles of cavity for PCR 1
 *
 * Pin definitions:
 *   The six cavity pins of two PCRs correspond to the encoder pins: {{{2,3},{4,5},{6,7}}, {{8,9},{10,11},{12,13}}} in the order {SCK,SDA}
 *   3 sucker pins {22,23,24}
 *   DAC pins {50,51,52,53}
*/
inline void communicator::control_cavity(
    const DTYPE angle0,
    const DTYPE angle1,
    const DTYPE angle2) {

    std::string command;
    command = (std::string)("C,") + std::to_string(angle0) + "," + \
            std::to_string(angle1) + "," + std::to_string(angle2) + ","\
            "-1," + "-1," + "-1";
    blocking_send(command);
}

inline void communicator::control_cavity(
    const DTYPE angle0,
    const DTYPE angle1,
    const DTYPE angle2,
    const DTYPE angle3,
    const DTYPE angle4,
    const DTYPE angle5) {

    std::string command;
    command = (std::string)("C,") + \
            std::to_string(angle0) + "," + std::to_string(angle1) + "," + std::to_string(angle2) + "," + \
            std::to_string(angle3) + "," + std::to_string(angle4) + "," + std::to_string(angle5);
    blocking_send(command);
}

inline void communicator::control_sucker(
    const char status0,
    const char status1) {

    std::string command;
    command = (std::string)("S,") + status0 + "," + status1;
    blocking_send(command);
}

inline void communicator::control_sucker(
    const char status0,
    const char status1,
    const char status2) {

    std::string command;
    command = (std::string)("S,") + status0 + "," + status1 + ",", status2;
    blocking_send(command);
}

inline void communicator::config(const int n_section, const int mode) {
    std::string command;
    command = std::to_string(n_section) + "," + std::to_string(mode);
    blocking_send(command);
}

#endif
