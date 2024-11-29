#include "hardware_interface.hpp"

// Define motor pins
#define FL_MOTOR_EN 9
#define FL_MOTOR_CW 8
#define FL_MOTOR_CLK 7

#define FR_MOTOR_EN 6
#define FR_MOTOR_CW 5
#define FR_MOTOR_CLK 4

#define RL_MOTOR_EN 3
#define RL_MOTOR_CW 2
#define RL_MOTOR_CLK 1

#define RR_MOTOR_EN 12
#define RR_MOTOR_CW 11
#define RR_MOTOR_CLK 10

HardwareInterface hardwareInterface;
static unsigned long lastFeedbackTime = 0;

void setup() {
    Serial.begin(115200);

    // Create motor objects
    Motor* flMotor = new UnipolarMotor(8, 10, 9, 11,"front_left_wheel_joint");//new DriverMotor(FL_MOTOR_EN, FL_MOTOR_CW, FL_MOTOR_CLK,"front_left_wheel_joint");
    //Motor* frMotor = new UnipolarMotor(32, 33, 34, 35, "front_right_wheel_joint");//new DriverMotor(FR_MOTOR_EN, FR_MOTOR_CW, FR_MOTOR_CLK,"front_right_wheel_joint");
    //Motor* rrMotor = new UnipolarMotor(12, 50, 45, 40, "back_left_wheel_joint");//new DriverMotor(RR_MOTOR_EN, RR_MOTOR_CW, RR_MOTOR_CLK,"rear_right_wheel_joint");
    //Motor* rlMotor = new UnipolarMotor(14, 15, 16, 17, "back_right_wheel_joint");//new DriverMotor(RL_MOTOR_EN, RL_MOTOR_CW, RL_MOTOR_CLK,"rear_left_wheel_joint");
    
    

    // Add motors to hardware interface
    hardwareInterface.addMotor(flMotor);
 //   hardwareInterface.addMotor(frMotor);
  //  hardwareInterface.addMotor(rlMotor);
   // hardwareInterface.addMotor(rrMotor);

    // Setup all motors
    hardwareInterface.setup();

    //Serial.println("Motors initialized and ready to receive commands!");
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

