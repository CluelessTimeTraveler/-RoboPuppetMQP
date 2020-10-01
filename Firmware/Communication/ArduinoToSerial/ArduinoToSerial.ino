




int sensorPin = 32;    // select the input pin for the potentiometer
int sensorValue;  // variable to store the value coming from the sensor




uint16_t toSend[9];
void setup() {

  //Serial.begin(9600);
  pinMode(sensorPin, INPUT);
  Serial.begin(9600);
  
}
void loop() {
  // read the value from the sensor:
  //sensorValue = analogRead(sensorPin);
  toSend[0] = analogRead(sensorPin);
  toSend[1] = 10;
  toSend[2] = 20;
  toSend[3] = 30;
  toSend[4] = 40;
  toSend[5] = 50;
  toSend[6] = 60;
  toSend[7] = 70;
  toSend[8] = 80;
  toSend[9] = 90;
  
  String servoCompressed = String(toSend[0]) + ',' + String(toSend[1]) + ',' + String(toSend[2]);
  String encoderCompressed = String(toSend[3]) + ',' + String(toSend[4]) + ',' + String(toSend[5]) + ',' + String(toSend[6]) + ',' + String(toSend[7]);
  String otherData = String(toSend[8]) + ',' + String(toSend[9]);
  Serial.println(servoCompressed + ',' + encoderCompressed + ',' + otherData);
  //Serial.write((uint8_t*)toSend, sizeof(toSend));
  delay(1000);
}
