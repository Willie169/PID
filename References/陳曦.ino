int orange1 = 11;
int blue1 = 9;
int orange2 = 5;
int blue2 = 3;
int trigpin_left = 13;
int echopin_left = 12;
int trigpin_right = 8;
int echopin_right = 7;
long duration;
float dist_left, dist_right, lr_dif;
float rkp = 1.0;
float rki = 0.01;
float rkd = 0.01;
float mkp = 1.0;
float mki = 0.1;
float mkd = 0.01;
float dt;
float rsetpoint = 0;
float rpe = 0;
float mpe = 0;
float mintegral = 0;
float rintegral = 0;
float rotation_factor = 100;
float move_factor = 10;
float proper_dist = 100;
int setrpm = 180;
unsigned long previousTime, mpreviousTime;

void setup()
{
 
  Serial.begin (9600);
  pinMode((orange1,blue1,orange2,blue2),OUTPUT);
  pinMode(trigpin_left, OUTPUT);
  pinMode(trigpin_right, OUTPUT);
  pinMode(echopin_left, INPUT);
  pinMode(echopin_right, INPUT);
  previousTime = millis();
  mpreviousTime = millis();

}

void loop()
{
  
  //left sensor
  digitalWrite(trigpin_left, LOW);
  delayMicroseconds(5);
  digitalWrite(trigpin_left, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin_left, LOW);
  pinMode(echopin_left, INPUT); 
  duration = pulseIn(echopin_left, HIGH); 
  dist_left = (duration/2) / 29.1;
  
  //right sensor
  digitalWrite(trigpin_right, LOW);
  delayMicroseconds(5);
  digitalWrite(trigpin_right, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin_right, LOW);
  pinMode(echopin_right, INPUT); 
  duration = pulseIn(echopin_right, HIGH); 
  dist_right = (duration/2) / 29.1;
  
  //calculate objective angle
  lr_dif = dist_left - dist_right;
  
  //print
  Serial.print("dist_left: "); Serial.println(dist_left);
  Serial.print("dist_right: "); Serial.println(dist_right);
  Serial.print("lr_dif: "); Serial.println(lr_dif);
  
  //Rotation PID 
  unsigned long currentTime = millis();
  dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;
  
  float error = lr_dif;
  rintegral += error * dt;
  float derivative = (error - rpe) / dt;
  rpe = error;
  float output = rkp * error + rki * rintegral + rkd * derivative;
  output = output * rotation_factor;//needs to be tuned
  
  //Rotation
  if(abs(error) > 0){
    if(output < 0)
      left(5000);
    else
      right(5000);  
  }
  
  //Move PID
  unsigned long mcurrentTime = millis();
  dt = (mcurrentTime - mpreviousTime) / 1000.0;
  mpreviousTime = mcurrentTime;
  
  if(abs(error) < 2){
  	float m_measurement = dist_left;
    float m_error = proper_dist - m_measurement;
    mintegral += m_error * dt;
    float m_derivative = (m_error - mpe) / dt;
    mpe = m_error;
    float m_output = mkp * m_error + mki * mintegral + rkd * m_derivative;
    m_output = m_output * move_factor;
    
    //Move
    if(m_output > 0)
      back(5000);
    else
      forward(5000);
  }
  
  
  
}

// Function for forward movement with RPM control
void forward(int rpm) {
  analogWrite(orange1, 0);
  analogWrite(blue1, rpm);
  analogWrite(orange2, 0);
  analogWrite(blue2, rpm);
  delayMicroseconds(10000);//can try rpm control
  stop();
}

// Function for backward movement with RPM control
void back(int rpm) {
  analogWrite(orange1, rpm);
  analogWrite(blue1, 0);
  analogWrite(orange2, rpm);
  analogWrite(blue2, 0);
  delayMicroseconds(10000);
  stop();
}

// Function for left turn with RPM control
void left(int rpm) {
  analogWrite(orange1, 0);
  analogWrite(blue1, rpm);
  analogWrite(orange2, rpm);
  analogWrite(blue2, 0);
  delayMicroseconds(10000);
  stop();
}

// Function for right turn with RPM control
void right(int rpm) {
  analogWrite(orange1, rpm);
  analogWrite(blue1, 0);
  analogWrite(orange2, 0);
  analogWrite(blue2, rpm);
  delayMicroseconds(10000);
  stop();
}

// Function to stop the car
void stop() {
  analogWrite(orange1, 0);
  analogWrite(blue1, 0);
  analogWrite(orange2, 0);
  analogWrite(blue2, 0);
}
