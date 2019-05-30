void setup() {
  Serial.begin(115200); // opens serial port, sets data rate to 57600 baud
}
 
void loop() {
  static const int ABSOLUTE_ENCODER_PIN = A0;
  static const uint16_t NUM_SAMPLES = 50;
  static const uint16_t GEAR_REDUCTION = 40; //40:1 reduction 
  
  static double position = 0;
  static double oldEncoderReading = 0;
  static double encoderReading = 0;

  encoderReading = analogRead(ABSOLUTE_ENCODER_PIN) * 360.0 / 1024.0;
  
  if(fabs(encoderReading - oldEncoderReading) < 180) { 
    position += (encoderReading - oldEncoderReading)/GEAR_REDUCTION;
  } else if (encoderReading < oldEncoderReading) {
    //rollover
    position += (encoderReading - oldEncoderReading + 360)/GEAR_REDUCTION;
  } else {
    //rollunder
    position += (encoderReading - oldEncoderReading - 360)/GEAR_REDUCTION;
  }

  Serial.println(position);
  
  oldEncoderReading = encoderReading;
}
