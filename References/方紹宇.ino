const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

// Define the stepper motors and the pins the will use
const int IN1 = 12;          //L298N IN1 pin 連接 Arduino pin 10
const int IN2 = 11;          //L298N IN2 pin 連接 Arduino pin 11
const int IN3 = 10;           //L298N IN3 pin 連接 Arduino pin 5
const int IN4 = 9;            //L298N IN4 pin 連接 Arduino pin 6
const int ENA = 8;
const int ENB = 13;

// Define pins for ultrasonic sensor
const int trigPinR = 4;
const int echoPinR = 5;
const int trigPinL = 2;
const int echoPinL = 3;
int duration1, distance1, duration2, distance2;
float pValue = 2, spd1, spd2;

void setup() {

  // Set initial seed values for the steppers
  pinMode(IN1, OUTPUT);      // Arduino 輸出電壓控制車子
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT); 
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);

  Serial.begin(9600);
  delay(500);
}

void loop() {

  // Trigger the ultrasonic sensor on the right
  digitalWrite(trigPinR, LOW);  // Make sure the trigger is low
  delayMicroseconds(2);  // Small delay
  digitalWrite(trigPinR, HIGH);  // Trigger the pulse
  delayMicroseconds(10);  // Pulse duration
  digitalWrite(trigPinR, LOW);  // End the pulse

  // Read the echo pin for right sensor
  duration1 = pulseIn(echoPinR, HIGH);  // Measure pulse duration
  distance1 = duration1 * 0.034 / 2;  // Calculate distance in cm

  // Trigger the ultrasonic sensor on the left
  digitalWrite(trigPinL, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);

  // Read the echo pin for left sensor
  duration2 = pulseIn(echoPinL, HIGH);
  distance2 = duration2 * 0.034 / 2;  // Calculate distance in cm

  //120-250
  if( (distance1-20)>15 || (distance1-20)<-15 ){
    spd1 = 0;
  }else if( (distance1-20)>=0 ){
    spd1 = (distance1-20) *100/15 + 100;
  }else{
    spd1 = (distance1-20) *100/15 - 100;
  }

  if( (distance2-20)>15 || (distance2-20)<-15 ){
    spd2 = 0;
  }else if( (distance2-20)>=0 ){
    spd2 = (distance2-20) *100/15 + 100;
  }else{
    spd2 = (distance2-20) *100/15 - 100;
  }

  // Print distances to Serial Monitor for debugging
  Serial.print("Right: ");
  Serial.print(distance1);
  Serial.print(", ");
  Serial.print(spd1);
  Serial.print("    ");
  Serial.print("Left: ");
  Serial.print(distance2);
  Serial.print(", ");
  Serial.print(spd2);
  Serial.print('\n');

  // Run the motors
  if(spd1>=0){
    analogWrite(ENA, spd1);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }else{
    analogWrite(ENA, spd1*-1);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  if(spd2>=0){
    analogWrite(ENB, spd2);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }else{
    analogWrite(ENB, spd2*-1);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

}
