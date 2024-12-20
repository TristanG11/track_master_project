/*#ifndef PINSCONFIG_HPP
#define PINSCONFIG_HPP

// Broches pour le moteur avant droit (Front Right)
const int MOTOR_FR_DIR_PIN = 2;        // Direction
const int MOTOR_FR_ENABLE_PIN = 3;    // Activation
const int MOTOR_FR_ENCODER_A_PIN = 18; // Encodeur A
const int MOTOR_FR_ENCODER_B_PIN = 19; // Encodeur B
const int MOTOR_FR_CURRENT_PIN = A0;  // Mesure du courant (analogique)
const int MOTOR_FR_VOLTAGE_PIN = A1;  // Mesure de la tension (analogique)

// Broches pour le moteur avant gauche (Front Left)
const int MOTOR_FL_DIR_PIN = 4;        // Direction
const int MOTOR_FL_ENABLE_PIN = 5;    // Activation
const int MOTOR_FL_ENCODER_A_PIN = 20; // Encodeur A
const int MOTOR_FL_ENCODER_B_PIN = 21; // Encodeur B
const int MOTOR_FL_CURRENT_PIN = A2;  // Mesure du courant (analogique)
const int MOTOR_FL_VOLTAGE_PIN = A3;  // Mesure de la tension (analogique)

// Broches pour le moteur arrière droit (Rear Right)
const int MOTOR_RR_DIR_PIN = 6;        // Direction
const int MOTOR_RR_ENABLE_PIN = 7;    // Activation
const int MOTOR_RR_ENCODER_A_PIN = 22; // Encodeur A
const int MOTOR_RR_ENCODER_B_PIN = 23; // Encodeur B
const int MOTOR_RR_CURRENT_PIN = A4;  // Mesure du courant (analogique)
const int MOTOR_RR_VOLTAGE_PIN = A5;  // Mesure de la tension (analogique)

// Broches pour le moteur arrière gauche (Rear Left)
const int MOTOR_RL_DIR_PIN = 8;        // Direction
const int MOTOR_RL_ENABLE_PIN = 9;    // Activation
const int MOTOR_RL_ENCODER_A_PIN = 24; // Encodeur A
const int MOTOR_RL_ENCODER_B_PIN = 25; // Encodeur B
const int MOTOR_RL_CURRENT_PIN = A6;  // Mesure du courant (analogique)
const int MOTOR_RL_VOLTAGE_PIN = A7;  // Mesure de la tension (analogique)


const int BATT_CURRENT_PIN = A8;
const int BATT_VOLTAGE_PIN = A9;
#endif // PINSCONFIG_HPP
*/
#ifndef PINSCONFIG_HPP
#define PINSCONFIG_HPP

// Mappage des broches pour l'ESP8266
// Assurez-vous que ces GPIO sont disponibles sur votre carte et non utilisés par d'autres fonctions importantes

// Moteur avant droit (Front Right)
const int MOTOR_FR_DIR_PIN = 5;         // GPIO5
const int MOTOR_FR_ENABLE_PIN = 4;      // GPIO4
const int MOTOR_FR_ENCODER_A_PIN = 0;   // GPIO0
const int MOTOR_FR_ENCODER_B_PIN = 2;   // GPIO2
const int MOTOR_FR_CURRENT_PIN = A0;    // Unique pin analogique sur ESP8266
const int MOTOR_FR_VOLTAGE_PIN = 14;    // GPIO14

// Moteur avant gauche (Front Left)
const int MOTOR_FL_DIR_PIN = 12;        // GPIO12
const int MOTOR_FL_ENABLE_PIN = 13;     // GPIO13
const int MOTOR_FL_ENCODER_A_PIN = 15;  // GPIO15
const int MOTOR_FL_ENCODER_B_PIN = 16;  // GPIO16
const int MOTOR_FL_CURRENT_PIN = A0;    // Partage du pin analogique
const int MOTOR_FL_VOLTAGE_PIN = 14;    // GPIO14

// Moteur arrière droit (Rear Right)
const int MOTOR_RR_DIR_PIN = 13;         // GPIO3
const int MOTOR_RR_ENABLE_PIN = 12;      // GPIO1
const int MOTOR_RR_ENCODER_A_PIN = 9;   // GPIO9
const int MOTOR_RR_ENCODER_B_PIN = 10;  // GPIO10
const int MOTOR_RR_CURRENT_PIN = A0;    // Partage du pin analogique
const int MOTOR_RR_VOLTAGE_PIN = 14;    // GPIO14

// Moteur arrière gauche (Rear Left)
const int MOTOR_RL_DIR_PIN = 2;         // GPIO2
const int MOTOR_RL_ENABLE_PIN = 0;      // GPIO0
const int MOTOR_RL_ENCODER_A_PIN = 4;   // GPIO4
const int MOTOR_RL_ENCODER_B_PIN = 5;   // GPIO5
const int MOTOR_RL_CURRENT_PIN = A0;    // Partage du pin analogique
const int MOTOR_RL_VOLTAGE_PIN = 14;    // GPIO14

// Batterie
const int BATT_CURRENT_PIN = A0;        // Mesure du courant (unique ADC sur ESP8266)
const int BATT_VOLTAGE_PIN = A0;        // Mesure de la tension (partage)

#endif // PINSCONFIG_HPP
