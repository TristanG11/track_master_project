#include "Arduino.h"
#include "common.hpp"


MultiplexerCode::MultiplexerCode(unsigned int S0, unsigned int S1, unsigned int S2, unsigned int S3): S0(S0), S1(S1), S2(S2), S3(S3s) {}
void MultiplexerCode::setChannel() {
  digitalWrite(PinS0, S0);
  digitalWrite(PinS1, S1);
  digitalWrite(PinS2, S2);
  digitalWrite(PinS3, S3);
}

