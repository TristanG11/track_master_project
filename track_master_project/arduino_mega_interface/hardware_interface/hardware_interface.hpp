#ifndef HARDWARE_INTERFACE_HPP
#define HARDWARE_INTERFACE_HPP

#include <ArxContainer.h>   // Include ArduinoSTL for standard C++ library support
#include "motor_interface.hpp"
#include "unipolar_motor.hpp"
#include "driver_motor.hpp"


class HardwareInterface {
public:
    HardwareInterface();
    void setup();
    void handleCommand(const String& cmd);
    void addMotor(Motor* motor);
    void run();
    void feedback();

private:
    std::map<String, Motor*> motors_; // Map of motor IDs to Motor pointers
};

#endif // HARDWARE_INTERFACE_HPP
