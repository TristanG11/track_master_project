// Floats for resistor values in divider (in ohms)
const float R1 = 30000.0;
const float R2 = 7500.0; 
const float voltage_factor = (R1+R2)/R2;
const float voltage_resolution = 5.0 / 1023.0; // Résolution ADC (en volts)
const float offset_voltage = 2.5;              // Offset du capteur de courant (typique)
const float sensitivity = 0.185;   


class MultiplexerCode{
  public:
    unsigned int S0;
    unsigned int S1;
    unsigned int S2;
    unsigned int S3;

    MultiplexerCode(unsigned int S0, unsigned int S1, unsigned int S2, unsigned int S3);
    void setChannel();
};


// Pins config : 

const int PinS0 = 3;
const int PinS1 = 4;
const int PinS2 = 5;
const int PinS3 = 6;

// Pins configuration for multiplexer control

MultiplexerCode voltageMotorFrontLeft(0, 0, 0, 0);    // Canal 0
MultiplexerCode currentMotorFrontLeft(0, 0, 0, 1);    // Canal 1
MultiplexerCode voltageMotorFrontRight(0, 0, 1, 0);   // Canal 2
MultiplexerCode currentMotorFrontRight(0, 0, 1, 1);   // Canal 3
MultiplexerCode voltageMotorRearLeft(0, 1, 0, 0);     // Canal 4
MultiplexerCode currentMotorRearLeft(0, 1, 0, 1);     // Canal 5
MultiplexerCode voltageMotorRearRight(0, 1, 1, 0);    // Canal 6
MultiplexerCode currentMotorRearRight(0, 1, 1, 1);    // Canal 7
MultiplexerCode batteryVoltage(1, 0, 0, 0);           // Canal 8
MultiplexerCode batteryCurrent(1,0,0,1);              // Canal 9
