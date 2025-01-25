#ifndef PIN_CONFIG_HPP
#define PIN_CONFIG_HPP

#include <Arduino.h> // Nécessaire pour le type uint8_t

// Configuration des pins pour les moteurs
// Chaque moteur utilise : 3 pins (CLK, EN, STEPDIR) + 2 analogiques (tension, courant)

struct MotorPins {
    uint8_t clk;       // Pin pour le signal CLK
    uint8_t en;        // Pin pour l'activation EN
    uint8_t cw;   // Pin pour 
    uint8_t voltage;   // Pin analogique pour mesurer la tension
    uint8_t current;   // Pin analogique pour mesurer le courant
};

// Définition des moteurs (exemple avec 4 moteurs : fl, fr, rl, rr)
const MotorPins MOTOR_FL = {2, 3, 4, A0, A1};  // Front Left
const MotorPins MOTOR_FR = {5, 6, 7, A2, A3};  // Front Right
const MotorPins MOTOR_RL = {8, 9, 10, A4, A5}; // Rear Left
const MotorPins MOTOR_RR = {11, 12, 13, A6, A7}; // Rear Right

// Configuration des pins pour la batterie (tension et courant)
struct BatteryPins {
    uint8_t voltage;   // Pin analogique pour la tension de la batterie
    uint8_t current;   // Pin analogique pour le courant de la batterie
};

// Définition des pins de la batterie
const BatteryPins BATTERY = {A8, A9}; // Pins analogiques pour la batterie

#endif // PIN_CONFIG_HPP
