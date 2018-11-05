#include <RedBot.h>

RedBotMotors motors;
RedBotSensor leftSensor = RedBotSensor(A3);
RedBotSensor centerSensor = RedBotSensor(A6);
RedBotSensor rightSensor = RedBotSensor(A7);
RedBotEncoder encoder = RedBotEncoder(A2, 10);
const int commandSize = 15;

char rxBuffer[commandSize], txBuffer[commandSize];
int i = 0;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int mode = 1;
int available = 0;
long leftEncoderCounter, rightEncoderCounter;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
}

void loop() {
  if(Serial.available() >= commandSize) {
    Serial.readBytes(rxBuffer, commandSize);
    processCommand();
  }
}

void processCommand() {
  // now we should parse the recieved command
  if(rxBuffer[0] == 0x01) {
    // so it's a command
    switch(rxBuffer[1]) {
      case 0x01: // setLeftMotorSpeed
        leftMotorSpeed = rxBuffer[2];
        if(rxBuffer[3] == 0) leftMotorSpeed = -leftMotorSpeed;
        motors.leftMotor(leftMotorSpeed);
      break;
      case 0x02: // setRightMotorSpeed
        rightMotorSpeed = rxBuffer[2];
        if(rxBuffer[3] == 0) rightMotorSpeed = -rightMotorSpeed;
        motors.rightMotor(rightMotorSpeed);
      break;
      case 0x03: // setMode
        mode = rxBuffer[2];
      break;
      case 0x04: // readSensorsData
        leftEncoderCounter = encoder.getTicks(LEFT); 
//        leftEncoderCounter = 0x12345678;
        // this is long value (4bytes) (0x12345678)Hex =  (305419896)Dec
        rightEncoderCounter = encoder.getTicks(RIGHT);
//        rightEncoderCounter = 0x31255441;
        // this is long value (4bytes) (0x31255441)Hex =  (824529985)Dec

        for(i=0;i<commandSize;i++){
          txBuffer[i] = 0x00;
        }
        
        txBuffer[0] = 0x02;
        txBuffer[1] = 0x01;
        // set left encoder value (big endian)
        txBuffer[2] = (leftEncoderCounter >> 24) & 0xFF;
        txBuffer[3] = (leftEncoderCounter >> 16) & 0xFF;
        txBuffer[4] = (leftEncoderCounter >> 8) & 0xFF;
        txBuffer[5] = leftEncoderCounter & 0xFF;

        // set right encoder value (big endian)
        txBuffer[6] = (rightEncoderCounter >> 24) & 0xFF;
        txBuffer[7] = (rightEncoderCounter >> 16) & 0xFF;
        txBuffer[8] = (rightEncoderCounter >> 8) & 0xFF;
        txBuffer[9] = rightEncoderCounter & 0xFF;

        // set IR data
        txBuffer[10] = leftSensor.read();
        txBuffer[11] = centerSensor.read();
        txBuffer[12] = rightSensor.read();

        // set collider data
        txBuffer[13] = 0x00;
        txBuffer[14] = 0x00;
//        
//        for(i=(sizeof(txBuffer)/sizeof(*txBuffer));i<commandSize;i++) {
//          txBuffer[i] = i;
//        }

        for(i=0;i<commandSize;i++){
          Serial.write((byte)txBuffer[i]);
          Serial.flush();
        }
      break;
    }
  }
}

void togglLed(int time) {
  for(i=0;i<5;i++){
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(time);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(time);  
  }
}