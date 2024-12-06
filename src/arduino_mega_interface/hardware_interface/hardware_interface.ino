#include "hardware_interface.hpp"


HardwareInterface hardwareInterface;
static unsigned long lastFeedbackTime = 0;

void setup() {
    Serial.begin(115200);

    // Create motor objects
    Motor* flMotor = new UnipolarMotor(8, 10, 9, 11,"front_left_wheel_joint");//new DriverMotor(FL_MOTOR_EN, FL_MOTOR_CW, FL_MOTOR_CLK,"front_left_wheel_joint");
    
    // Setup all motors

    hardwareInterface.setup();

}

void loop() {
    // Listen for commands via Serial
    
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n'); // Read the command
        hardwareInterface.handleCommand(cmd);     // Process the command
    }



    // Run the motors
    hardwareInterface.run();

    if (millis() - lastFeedbackTime > 200) {
        hardwareInterface.feedback();
        lastFeedbackTime = millis();
    }
}

