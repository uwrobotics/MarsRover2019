#define inA 13
#define inB 9
#define motorSpeed 4
String dataStr;
int speed;

void setup() {
  pinMode(inA, OUTPUT);
  pinMode(inB, OUTPUT);
  pinMode(motorSpeed, OUTPUT);
}

void loop() {
  // read string in from serial
  while (Serial.available() > 0) {
    dataStr = Serial.readString();

    // extract data from input
    // 1st bit = valid or not -> 0 means turn off motor, 1 means motor will be on
    // 2nd bit = direction (0 = CW, 1 = CCW)
    // rest of bits = speed (b/t 0 and 255)

    if (dataStr[0] == '0')
    {
      analogWrite(motorSpeed, 0);
      digitalWrite(inA, LOW);
      digitalWrite(inB, LOW);
    }
    else {
      // get speed as int
      speed = dataStr.substring(2, 4).toInt();
      
      // turn motor CW
      if (dataStr[1] = '0') {
        digitalWrite(inB, LOW);
        digitalWrite(inA, HIGH);
        
        analogWrite(motorSpeed, speed);
      }
      // otherwise turn CCW
      else {
        digitalWrite(inA, LOW);
        digitalWrite(inB, HIGH);
        
        analogWrite(motorSpeed, speed);
      }
    }
  }
}

