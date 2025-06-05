const int ENA = 3;  // Right motor PWM
const int IN1 = 4;  // Right motor direction
const int IN2 = 5;

const int ENB = 9;  // Left motor PWM
const int IN3 = 7;  // Left motor direction
const int IN4 = 6;

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Stop motors initially
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void loop() {
  if (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    // Expected format: "leftPWM,leftDir,rightPWM,rightDir\n"
    int values[4] = {0, 0, 0, 0};
    int valueIndex = 0;
    int lastComma = -1;
    int commaCount = 0;

    for (int i = 0; i < inputString.length(); i++) {
      if (inputString.charAt(i) == ',') {
        String valStr = inputString.substring(lastComma + 1, i);
        values[valueIndex] = valStr.toInt();
        valueIndex++;
        lastComma = i;
        commaCount++;
      }
    }
    // Last value after last comma
    String valStr = inputString.substring(lastComma + 1);
    values[valueIndex] = valStr.toInt();

    if (commaCount == 3) {
      int leftPWM = constrain(values[0], 0, 255);
      int leftDir = values[1] == 0 ? 0 : 1;
      int rightPWM = constrain(values[2], 0, 255);
      int rightDir = values[3] == 0 ? 0 : 1;

      // Control Left motor
      if (leftDir == 1) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
      } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
      }
      analogWrite(ENB, leftPWM);

      // Control Right motor
      if (rightDir == 1) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
      } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
      }
      analogWrite(ENA, rightPWM);
    }

    // Reset input buffer
    inputString = "";
    stringComplete = false;
  }
}
