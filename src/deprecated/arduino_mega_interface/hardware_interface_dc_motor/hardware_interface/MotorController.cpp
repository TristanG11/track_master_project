#include "Arduino.h"
#include "MotorController.hpp"


// Définition des membres statiques
Motor* MotorController::motor_fr = nullptr;
Motor* MotorController::motor_fl = nullptr;
Motor* MotorController::motor_rr = nullptr;
Motor* MotorController::motor_rl = nullptr;


// Constructeur
MotorController::MotorController() : battery_status(BATT_CURRENT_PIN,BATT_VOLTAGE_PIN) {
    motor_fr = new Motor(MOTOR_FR_DIR_PIN, MOTOR_FR_ENABLE_PIN, MOTOR_FR_ENCODER_A_PIN, MOTOR_FR_ENCODER_B_PIN,
                     "fr", MOTOR_FR_CURRENT_PIN, MOTOR_FR_VOLTAGE_PIN);
    motor_fl = new Motor(MOTOR_FL_DIR_PIN, MOTOR_FL_ENABLE_PIN, MOTOR_FL_ENCODER_A_PIN, MOTOR_FL_ENCODER_B_PIN,
                        "fl", MOTOR_FL_CURRENT_PIN, MOTOR_FL_VOLTAGE_PIN);
    motor_rr = new Motor(MOTOR_RR_DIR_PIN, MOTOR_RR_ENABLE_PIN, MOTOR_RR_ENCODER_A_PIN, MOTOR_RR_ENCODER_B_PIN,
                        "rr", MOTOR_RR_CURRENT_PIN, MOTOR_RR_VOLTAGE_PIN);
    motor_rl = new Motor(MOTOR_RL_DIR_PIN, MOTOR_RL_ENABLE_PIN, MOTOR_RL_ENCODER_A_PIN, MOTOR_RL_ENCODER_B_PIN,
                        "rl", MOTOR_RL_CURRENT_PIN, MOTOR_RL_VOLTAGE_PIN);

}

MotorController::~MotorController() {
    delete motor_fr;
    delete motor_fl;
    delete motor_rr;
    delete motor_rl;
}

void MotorController::init(){
  motor_fr->init();
  motor_fl->init();
  motor_rr->init();
  motor_rl->init();
}

// Méthodes statiques pour gérer les interruptions
void MotorController::handleMotorFrEncoderA() {
    if (motor_fr) {
        motor_fr->handleEncoderA();
    }
}

void MotorController::handleMotorFlEncoderA() {
    if (motor_fl) {
        motor_fl->handleEncoderA();
    }
}

void MotorController::handleMotorRrEncoderA() {
    if (motor_rr) {
        motor_rr->handleEncoderA();
    }
}

void MotorController::handleMotorRlEncoderA() {
    if (motor_rl) {
        motor_rl->handleEncoderA();
    }
}

// Attachement des interruptions
void MotorController::attachInterrupts() {
    attachInterrupt(digitalPinToInterrupt(MOTOR_FR_ENCODER_A_PIN), handleMotorFrEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_FL_ENCODER_A_PIN), handleMotorFlEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RR_ENCODER_A_PIN), handleMotorRrEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RL_ENCODER_A_PIN), handleMotorRlEncoderA, CHANGE);
}

const float INTEGRAL_LIMIT = 1000.0; // Définissez une limite adaptée à votre application

void MotorController::computePID() {
    // Moteur avant droit (FR)
    error_fr = motor_fr->getDesiredSpeed() - motor_fr->computeSpeed();
    integral_fr += error_fr;
    integral_fr = constrain(integral_fr, -INTEGRAL_LIMIT, INTEGRAL_LIMIT); // Limitez l'intégrale
    derivative_fr = error_fr - lastError_fr;
    lastError_fr = error_fr;
    motor_fr->resetTicks();

    // Répétez pour les autres moteurs
    error_fl = motor_fl->getDesiredSpeed() - motor_fl->computeSpeed();
    integral_fl += error_fl;
    integral_fl = constrain(integral_fl, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    derivative_fl = error_fl - lastError_fl;
    lastError_fl = error_fl;
    motor_fl->resetTicks();

    error_rr = motor_rr->getDesiredSpeed() - motor_rr->computeSpeed();
    integral_rr += error_rr;
    integral_rr = constrain(integral_rr, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    derivative_rr = error_rr - lastError_rr;
    lastError_rr = error_rr;
    motor_rr->resetTicks();

    error_rl = motor_rl->getDesiredSpeed() - motor_rl->computeSpeed();
    integral_rl += error_rl;
    integral_rl = constrain(integral_rl, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    derivative_rl = error_rl - lastError_rl;
    lastError_rl = error_rl;
    motor_rl->resetTicks();

    // Calcul et application des commandes PID
    
    /*motor_rl->setCmd(kp_rl * error_rl + ki_rl * integral_rl + kd_rl * derivative_rl);
    motor_rr->setCmd(kp_rr * error_rr + ki_rr * integral_rr + kd_rr * derivative_rr);
    motor_fl->setCmd(kp_fl * error_fl + ki_fl * integral_fl + kd_fl * derivative_fl);
    motor_fr->setCmd(kp_fr * error_fr + ki_fr * integral_fr + kd_fr * derivative_fr);*/
}

/*void MotorController::computePID() {
    // Moteur avant droit (FR)
    error_fr = motor_fr->getDesiredSpeed() - motor_fr->computeSpeed();
    integral_fr += error_fr;
    derivative_fr = error_fr - lastError_fr;
    lastError_fr = error_fr;
    motor_fr->resetTicks();

    // Moteur avant gauche (FL)
    error_fl = motor_fl->getDesiredSpeed() - motor_fl->computeSpeed();
    integral_fl += error_fl;
    derivative_fl = error_fl - lastError_fl;
    lastError_fl = error_fl;
    motor_fl->resetTicks();

    // Moteur arrière droit (RR)
    error_rr = motor_rr->getDesiredSpeed() - motor_rr->computeSpeed();
    integral_rr += error_rr;
    derivative_rr = error_rr - lastError_rr;
    lastError_rr = error_rr;
    motor_rr->resetTicks();

    // Moteur arrière gauche (RL)
    error_rl = motor_rl->getDesiredSpeed() - motor_rl->computeSpeed();
    integral_rl += error_rl;
    derivative_rl = error_rl - lastError_rl;
    lastError_rl = error_rl;
    motor_rl->resetTicks();


    motor_rl->setCmd(kp_rl * error_rl + ki_rl * integral_rl + kd_rl * derivative_rl);
    motor_rr->setCmd(kp_rr * error_rr + ki_rr * integral_rr + kd_rr * derivative_rr);
    motor_fl->setCmd(kp_fl * error_fl + ki_fl * integral_fl + kd_fl * derivative_fl);
    motor_fr->setCmd(kp_fr * error_fr + ki_fr * integral_fr + kd_fr * derivative_fr);
}*/

/*void MotorController::handleCommand(const String& cmd) {
    // Exemple d'entrée : "front_left_wheel_joint,0.5;front_right_wheel_joint,-0.5;rear_left_wheel_joint,0.3;rear_right_wheel_joint,-0.3;"

    int start = 0;
    int end = cmd.indexOf(';'); // Trouver le premier ';'

    while (end != -1) {
        // Extraire chaque segment (par exemple, "front_left_wheel_joint,0.5")
        String segment = cmd.substring(start, end);

        // Trouver le séparateur entre l'ID du moteur et la vitesse
        int separator = segment.indexOf(',');
        if (separator != -1) {
            // Extraire l'ID du moteur et la vitesse
            String motorId = segment.substring(0, separator); // Exemple : "front_left_wheel_joint"
            String speedValueStr = segment.substring(separator + 1); // Exemple : "0.5"
            float speedValue = speedValueStr.toFloat() * 1 /WHEEL_RADIUS; // Convertir la vitesse en float

            // Identifier le moteur et définir la vitesse
            if (motorId == "fl") {
                motor_fl->setDesiredSpeed(speedValue);
            } else if (motorId == "fr") {
                motor_fr->setDesiredSpeed(speedValue);
            } else if (motorId == "rl") {
                motor_rl->setDesiredSpeed(speedValue);
            } else if (motorId == "rr") {
                motor_rr->setDesiredSpeed(speedValue);
            } else {
                Serial.print("Error : Motor ID not found: ");
                Serial.println(motorId);
            }
        } else {
            Serial.print("Error : Malformed segment: ");
            Serial.println(segment);
        }

        // Mettre à jour la position de départ et trouver le prochain ';'
        start = end + 1;
        end = cmd.indexOf(';', start);
    }
}*/

void MotorController::handleCommand(const char* cmd) {
    char buffer[64];
    strncpy(buffer, cmd, sizeof(buffer));
    buffer[sizeof(buffer) - 1] = '\0'; // S'assurer que la chaîne est nul-terminée
    char* token = strtok(buffer, ";");
    while (token != nullptr) {
        char motorId[3];
        char speedStr[16];
        float speedValue = 0.0;

        // Valider la présence du séparateur ':'
        char* separator = strchr(token, ':');
        if (separator == nullptr) {
            Serial.print("<Error : Missing separator in segment: ");
            Serial.print(token);
            Serial.print(cmd);
            Serial.println(">");
            token = strtok(nullptr, ";");
            continue; // Passer au segment suivant
        }

        // Découper la chaîne autour du séparateur ':'
        strncpy(motorId, token, separator - token); // Copier l'identifiant avant ':'
        motorId[2] = '\0'; // S'assurer que motorId est nul-terminé
        strncpy(speedStr, separator + 1, sizeof(speedStr) - 1); // Copier la valeur après ':'

        // Vérifier que motorId est valide
        if (strcmp(motorId, "fl") == 0 || strcmp(motorId, "fr") == 0 ||
            strcmp(motorId, "rl") == 0 || strcmp(motorId, "rr") == 0) {
            speedValue = atof(speedStr); // Convertir en float
            speedValue *= 1 / WHEEL_RADIUS; // Appliquer la conversion

            // Assigner la vitesse au moteur correspondant
            if (strcmp(motorId, "fl") == 0) {
                motor_fl->setDesiredSpeed(speedValue);
            } else if (strcmp(motorId, "fr") == 0) {
                motor_fr->setDesiredSpeed(speedValue);
            } else if (strcmp(motorId, "rl") == 0) {
                motor_rl->setDesiredSpeed(speedValue);
            } else if (strcmp(motorId, "rr") == 0) {
                motor_rr->setDesiredSpeed(speedValue);
            }
        } else {
            Serial.print("Error : Invalid motor ID: ");
            Serial.println(motorId);
        }

        token = strtok(nullptr, ";"); // Passer au segment suivant
    }
}




void MotorController::sendFeedback() {
    char feedback[128];
    char frPosition[10], frSpeed[10];
    char flPosition[10], flSpeed[10];
    char rrPosition[10], rrSpeed[10];
    char rlPosition[10], rlSpeed[10];

    // Convertir les floats en chaînes avec dtostrf
    dtostrf(motor_fr->getPosition(), 5, 2, frPosition);
    dtostrf(motor_fr->getSpeed(), 5, 2, frSpeed);

    dtostrf(motor_fl->getPosition(), 5, 2, flPosition);
    dtostrf(motor_fl->getSpeed(), 5, 2, flSpeed);

    dtostrf(motor_rr->getPosition(), 5, 2, rrPosition);
    dtostrf(motor_rr->getSpeed(), 5, 2, rrSpeed);

    dtostrf(motor_rl->getPosition(), 5, 2, rlPosition);
    dtostrf(motor_rl->getSpeed(), 5, 2, rlSpeed);

    // Remplacer les espaces initiaux par des zéros
    char* strings[] = {frPosition, frSpeed, flPosition, flSpeed, rrPosition, rrSpeed, rlPosition, rlSpeed};
    for (int i = 0; i < 8; i++) {
        if (strings[i][0] == ' ') {
            strings[i][0] = '0'; // Remplace l'espace par un zéro
        }
    }

    // Construire la chaîne de feedback
    snprintf(feedback, sizeof(feedback),
             "<fr,%s,%s;fl,%s,%s;rr,%s,%s;rl,%s,%s>\n",
             frPosition, frSpeed,
             flPosition, flSpeed,
             rrPosition, rrSpeed,
             rlPosition, rlSpeed);

    // Envoyer le feedback via Serial
    Serial.print(feedback);
}





void MotorController::sendStatus() {
    char status[256];
    battery_status.updateAll();
    motor_fr->status.updateAll();
    motor_fl->status.updateAll();
    motor_rr->status.updateAll();
    motor_rl->status.updateAll();
    

    snprintf(status, sizeof(status), "<%s%s%s%s%s>\n",
             motor_fr->status.serialize(),
             motor_fl->status.serialize(),
             motor_rr->status.serialize(),
             motor_rl->status.serialize(),
             battery_status.serialize());

    Serial.print(status);
}

