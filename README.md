# SPARC

#### Prerequisites

- ROS
- serial：Serial communication library

#### Platform

- Linux

#### Directory Structure

- ros_code/catkin：ROS code directory. Responsible for controllers, reading optrack coordinates, etc., and sending the angles of each chamber to Arduino.

  - src：Source Code

    - controller: Controller code

 - controller：Controller code

      - src: Directory for specific control implementation
        - gen_path.cpp：Generate reference path
        - keyboard_controller.cpp：Control motion based on keyboard input
        - main.cpp：Controller entry
        - pure_pursuit.cpp：Implementation of pure pursuit algorithm
      - include: Various classes and partial function
        - characterization.h：Implementation for calibrating length and angle relationship
        - cavity.h：Chamber class for length and angle conversion
        - common.h：Common variables and functions
        - communicator.h：Class for communication with Arduino
        - gen_path.h：Header file for generating reference path
        - keyboard_controller.h：Class for keyboard-controlled motion
        - logger.h：Output and save parameters during robot motion
        - MPCR.h：Parallel continuum robot class, mainly including kinematics and robot parameters
        - pure_pursuit.h：Pure pursuit controller class
        - sensor.h：Parse optrack data and perform coordinate system transformation
    - read_keyboard: Read keyboard content
    - read_sensor: Read optrack coordinates

- arduino：Arduino-side code. Reads the angles of each chamber sent by the host computer and controls them to the corresponding angle through a PID controller.
  - arduino.ino：Program entry
  - AS5600_IIC.cpp, AS5600_IIC.h：Code for angle Hall sensor
  - cavity.cpp, cavity.h：Chamber PID controller
  - DAC.cpp, DAC.h, LTC2664.h, LTC2664.cpp：DAC module control code
  - sucker.cpp, sucker.h：Foot suction cup control code
