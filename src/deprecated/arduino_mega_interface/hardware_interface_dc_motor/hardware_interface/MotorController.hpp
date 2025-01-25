#ifndef MOTORCONTROLLER_HPP
#define MOTORCONTROLLER_HPP

#include "Motor.hpp"
#include "PinsConfig.hpp"

class MotorController {
private:
    static Motor* motor_fr; // Pointeur statique vers le moteur avant droit
    static Motor* motor_fl; // Pointeur statique vers le moteur avant gauche
    static Motor* motor_rr; // Pointeur statique vers le moteur arrière droit
    static Motor* motor_rl; // Pointeur statique vers le moteur arrière gauche

    // Coefficients PID pour chaque moteur
    volatile float kp_fr, ki_fr, kd_fr; // PID pour moteur avant droit
    volatile float kp_fl, ki_fl, kd_fl; // PID pour moteur avant gauche
    volatile float kp_rr, ki_rr, kd_rr; // PID pour moteur arrière droit
    volatile float kp_rl, ki_rl, kd_rl; // PID pour moteur arrière gauche

    volatile float error_fr, integral_fr, derivative_fr, lastError_fr; // Variables PID
    volatile float error_fl, integral_fl, derivative_fl, lastError_fl;
    volatile float error_rr, integral_rr, derivative_rr, lastError_rr;
    volatile float error_rl, integral_rl, derivative_rl, lastError_rl;

    BatteryStatus battery_status;

public:
    MotorController();
    ~MotorController();
    void init();
    static void handleMotorFrEncoderA();
    static void handleMotorFlEncoderA();
    static void handleMotorRrEncoderA();
    static void handleMotorRlEncoderA();
    void handleCommand(const char* cmd);
    void attachInterrupts();
    void computePID();
    void sendFeedback();
    void sendStatus();
};

#endif // MOTORCONTROLLER_HPP
