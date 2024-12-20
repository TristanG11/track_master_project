#include "MotorController.hpp"
//#include <TimerThree.h>

MotorController motorController;

unsigned long lastFeedbackTime = 0;   // Dernière heure d'envoi du feedback
unsigned long lastStatusTime = 0;     // Dernière heure d'envoi du status
unsigned long lastMessageSentTime = 0; // Dernière heure d'envoi d'un message quelconque

unsigned long feedbackInterval = 40;   // Intervalle pour feedback en ms
unsigned long statusInterval = 1000;    // Intervalle pour status en ms
unsigned long interMessageDelay = 30;  // Temporisation minimale entre feedback et status en ms


// Déclaration d'une variable en fin de la section .bss
/*extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

int freeMemory() {
  int free_memory;
  if ((int)__brkval == 0) {
    // Si aucun malloc n'a été utilisé, l'espace libre est entre __bss_end et le haut de la pile.
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  } else {
    // Si malloc a été utilisé, l'espace libre est entre __brkval et le haut de la pile.
    free_memory = ((int)&free_memory) - ((int)__brkval);
  }
  return free_memory;
}*/

/*
char serialBuffer[50]; // Tampon pour stocker les commandes série

void processSerialInput() {
    while (Serial.available()) {
        char incomingChar = Serial.read(); // Lire un caractère
        if (incomingChar == '\n') {        // Fin de commande détectée
            motorController.handleCommand(serialBuffer); // Traiter la commande
            serialBuffer = "";             // Réinitialiser le tampon
        } else {
            serialBuffer += incomingChar;  // Ajouter au tampon
        }
    }
}
*/

// Configuration de TimerThree pour appeler computePID périodiquement
/*void setupTimer() {
    // Configurer TimerThree avec la fréquence en Hz
    Timer3.initialize(TIMER_FREQUENCY_SEC * 1000000); // Intervalle en microsecondes
    Timer3.attachInterrupt([]() {
        // Appeler computePID dans l'interruption
        motorController.computePID();
    });
}*/

#include <Ticker.h>
Ticker pidTicker;

void setupTimer() {
    pidTicker.attach_ms(TIMER_FREQUENCY_SEC * 1000, []() {
        motorController.computePID();
    });
}




void setup() {
    // Initialisation de la communication série
    Serial.begin(500000);
    
    //Serial.println(Serial.availableForWrite());
    //Serial.println(freeMemory());
    //Serial.println(Serial.availableForWrite());
    delay(2000);  // Pause pour stabiliser l'ESP
    
    // Initialisation des moteurs et interruptions
    motorController.init();
    //motorController.attachInterrupts();

    // Message de démarrage
    Serial.println("Motor Controller Initialized");
    setupTimer();
}

void loop() {
   unsigned long currentTime = millis();
    
    // Écouter les commandes via Serial
    if (Serial.available()>0) {
        //noInterrupts();  // Désactiver les interruptions
        String cmd = Serial.readStringUntil('\n'); // Lire la commande
        motorController.handleCommand(cmd.c_str());// Traiter la commande
        
        //interrupts();  // Réactiver les interruptions
    }


    


    
    // Envoi du feedback toutes les 40 ms
    if (currentTime - lastFeedbackTime >= feedbackInterval) {
        if (currentTime - lastMessageSentTime >= interMessageDelay) {
            //if(Serial.availableForWrite() > 55 ){
                motorController.sendFeedback();
            //}else{
              //  Serial.println("BOOOOOOOOOOOOOOOOOO");
            //}
            lastFeedbackTime = currentTime;
            lastMessageSentTime = currentTime; // Marquer l'heure d'envoi
        }
    }


    // Envoi du status toutes les 1000 ms
    if (currentTime - lastStatusTime >= statusInterval) {
        if (currentTime - lastMessageSentTime >= interMessageDelay) {
   //       Serial.print(freeMemory());
              //if(Serial.availableForWrite() > 100){
               motorController.sendStatus();
              //}else{
               // Serial.println("BOOOOOOOOOOOOOOOOOO");
              //}
            lastStatusTime = currentTime;
            lastMessageSentTime = currentTime; // Marquer l'heure d'envoi
        }
    }
}



