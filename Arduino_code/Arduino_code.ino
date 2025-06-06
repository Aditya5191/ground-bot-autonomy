// Motor Pins
const int ENA = 9;  // Left motor PWM
const int IN1 = 6;
const int IN2 = 7;

const int ENB = 3;  // Right motor PWM
const int IN3 = 4;
const int IN4 = 5;

void setup() {
  Serial.begin(9600);
  
  // Setup motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set both motors to move forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void loop() {
  if (Serial.available() >= 2) {
    int leftPWM = Serial.read();
    int rightPWM = Serial.read();

    analogWrite(ENA, leftPWM);
    analogWrite(ENB, rightPWM);

    // Optional: Print for debugging
    Serial.print("Left PWM: ");
    Serial.print(leftPWM);
    Serial.print(" | Right PWM: ");
    Serial.println(rightPWM);
  }
}
