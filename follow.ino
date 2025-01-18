#define IR1 5  // Left IR sensor
#define IR2 6  // Center IR sensor
#define IR3 7  // Right IR sensor

// Motor control pins
#define IN1 11
#define IN2 10
#define IN3 9
#define IN4 8

// PID constants
float Kp = 1.5;   // Proportional gain
float Ki = 0.0;   // Integral gain
float Kd = 0.4;   // Derivative gain

// PID variables
float error = 0, previous_error = 0;
float integral = 0, derivative = 0;
float correction = 0;

// Motor speed variable
int baseSpeed = 170;

void setup() {
  Serial.begin(9600);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // Read sensor values
  bool left = digitalRead(IR1);    // Left sensor
  bool center = digitalRead(IR2);  // Center sensor
  bool right = digitalRead(IR3);   // Right sensor

  // Calculate error based on sensor readings
  if (left == LOW && center == HIGH && right == LOW) {
    error = 0;  // On track
  }
  else if (left == LOW && center == LOW && right == HIGH) {
    error = -1;  // Turn left
  }
  else if (left == HIGH && center == LOW && right == LOW) {
    error = 1; // Turn right
  }

  // PID calculations
  integral += error;
  derivative = error - previous_error;
  correction = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;

  // Adjust motor speed based on PID correction
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  // Ensure motor speeds stay within the range (0-255)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Control motor speeds
  if (error == 0) {
    forward(leftSpeed+80); // Go straight
  } else if (error == 1) {
    spinRight(rightSpeed); // Turn right
  } else if (error == -1) {
    spinLeft(leftSpeed); // Turn left
  }
  delay(20); // Small delay to prevent sensor noise
}

void forward(int speed) {
  digitalWrite(IN1, HIGH);digitalWrite(IN2, LOW);analogWrite(IN1, speed);
  digitalWrite(IN3, HIGH);digitalWrite(IN4, LOW);analogWrite(IN3, speed);
}

void stop() {
  digitalWrite(IN1, LOW);digitalWrite(IN2, LOW);digitalWrite(IN3, LOW);digitalWrite(IN4, LOW);
}

void moveLeft(int speed) {
  digitalWrite(IN1, HIGH);analogWrite(IN1, speed);digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);digitalWrite(IN4, LOW);
}

void moveRight(int speed) {
  digitalWrite(IN1, LOW);digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);digitalWrite(IN4, LOW);analogWrite(IN3, speed);
}

void spinLeft(int speed) {
  digitalWrite(IN1, HIGH);analogWrite(IN1, speed);digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);digitalWrite(IN4, HIGH);analogWrite(IN4, speed);
}

void spinRight(int speed) {
  digitalWrite(IN1, LOW);digitalWrite(IN2, HIGH);analogWrite(IN2, speed);
  digitalWrite(IN3, HIGH);digitalWrite(IN4, LOW);analogWrite(IN3, speed);
}

void backward(int speed) {
  digitalWrite(IN1, LOW);digitalWrite(IN2, HIGH);analogWrite(IN1, speed);
  digitalWrite(IN3, LOW);digitalWrite(IN4, HIGH);analogWrite(IN1, speed);
}
