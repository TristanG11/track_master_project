#include "hardware_interface.hpp"


HardwareInterface hardwareInterface;
unsigned long lastFeedbackTime = 0;   // Dernière heure d'envoi du feedback
unsigned long lastStatusTime = 0;     // Dernière heure d'envoi du status
unsigned long lastMessageSentTime = 0; // Dernière heure d'envoi d'un message quelconque

unsigned long feedbackInterval = 40;   // Intervalle pour feedback en ms
unsigned long statusInterval = 1000;    // Intervalle pour status en ms
unsigned long interMessageDelay = 30;  // Temporisation minimale entre feedback et status en ms

void setup() {
    Serial.begin(115200);
    // Create motor objects
    Motor* flMotor = new DriverMotor(MOTOR_FL.en, MOTOR_FL.cw, MOTOR_FL.clk, "front_left_wheel_joint", MOTOR_FL.current, MOTOR_FL.voltage);
    Motor* frMotor = new DriverMotor(MOTOR_FR.en, MOTOR_FR.cw, MOTOR_FR.clk, "front_right_wheel_joint", MOTOR_FR.current, MOTOR_FR.voltage);
    Motor* rlMotor = new DriverMotor(MOTOR_RL.en, MOTOR_RL.cw, MOTOR_RL.clk, "rear_left_wheel_joint", MOTOR_RL.current, MOTOR_RL.voltage);
    Motor* rrMotor = new DriverMotor(MOTOR_RR.en, MOTOR_RR.cw, MOTOR_RR.clk, "rear_right_wheel_joint", MOTOR_RR.current, MOTOR_RR.voltage);

    // Setup all motors
    hardwareInterface.addMotor(flMotor);
    hardwareInterface.addMotor(frMotor);
    hardwareInterface.addMotor(rlMotor);
    hardwareInterface.addMotor(rrMotor);
    
    hardwareInterface.setup();
}
void loop() {
    unsigned long currentTime = millis();

    // Écouter les commandes via Serial
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n'); // Lire la commande
        hardwareInterface.handleCommand(cmd);     // Traiter la commande
    }

    // Faire fonctionner les moteurs
    hardwareInterface.run();

    // Envoi du feedback toutes les 40 ms
    if (currentTime - lastFeedbackTime >= feedbackInterval) {
        if (currentTime - lastMessageSentTime >= interMessageDelay) {
            hardwareInterface.feedback();
            lastFeedbackTime = currentTime;
            lastMessageSentTime = currentTime; // Marquer l'heure d'envoi
        }
    }

    // Envoi du status toutes les 100 ms
    if (currentTime - lastStatusTime >= statusInterval) {
        if (currentTime - lastMessageSentTime >= interMessageDelay) {
            hardwareInterface.sendStatus();
            lastStatusTime = currentTime;
            lastMessageSentTime = currentTime; // Marquer l'heure d'envoi
        }
    }



}