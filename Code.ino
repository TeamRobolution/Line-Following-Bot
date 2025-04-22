// Motor driver TB6612FNG pin definitions
int ENA = 10;  // PWM for Motor A (Right Motor)
int IN1 = 2;   // Direction control for Motor A
int IN2 = 3;   // Direction control for Motor A

int STBY = 4;  // Standby pin for TB6612FNG

int IN3 = 5;   // Direction control for Motor B (Left Motor)
int IN4 = 6;   // Direction control for Motor B
int ENB = 9;   // PWM for Motor B

// IR Sensor pin definitions
int Ir1 = A1;  // Left sensor
int Ir2 = A2;  // Slight left sensor (for PID)
int Ir3 = A3;  // Center sensor
int Ir4 = A4;  // Slight right sensor (for PID)
int Ir5 = A5;  // Right sensor

// PID variables
int P, I, D, PID, pre_P;
int Nor_spR = 145;  // Normal speed for the right motor
int Nor_spL = 190;  // Normal speed for the left motor
int sp_Rm, sp_Lm;   // Calculated speeds for the motors

float Kp = 0.8;
float Ki = 0.0;
float Kd = 2.2;

int minVal[5], maxVal[5], thres[5];  // Min, max, and threshold values for sensors

// Button pin definitions
int btnCalibrate = 11;     // Button for starting calibration
int btnLineFollow = 12;    // Button for starting line following

// Smoothing sensor readings
int smoothSensorRead(int pin) {
  int total = 0;
  for (int i = 0; i < 5; i++) {
    total += analogRead(pin);
  }
  return total / 5;  // Average over 5 readings
}

// Setup function to initialize pins
void setup() {
  // Motor pin setup
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Button pin setup
  pinMode(btnCalibrate, INPUT_PULLUP);  // Using internal pull-up resistor
  pinMode(btnLineFollow, INPUT_PULLUP); // Using internal pull-up resistor

  // Sensor pin setup
  pinMode(Ir1, INPUT);
  pinMode(Ir2, INPUT);
  pinMode(Ir3, INPUT);
  pinMode(Ir4, INPUT);
  pinMode(Ir5, INPUT);

  Serial.begin(9600);

}

void loop() {
  // Check if calibration button is pressed
  if (digitalRead(btnCalibrate) == LOW) {
    Serial.println("Starting calibration...");
    calibrate();
  }

  // Check if line-following button is pressed
  if (digitalRead(btnLineFollow) == LOW) {
    Serial.println("Starting line-following...");
    while (1) {
      lineFollowLoop();
    }
  }
}

// Calibration function
void calibrate() {
  for (int i = 0; i < 5; i++) {
    minVal[i] = analogRead(A1 + i);
    maxVal[i] = analogRead(A1 + i);
  }

  // Run for 500 cycles during calibration
  for (int i = 0; i < 50; i++) {
    // Pendulum-like motion to help sensors detect range
    if (i % 2 == 0) motorPendulumRight();
    else motorPendulumLeft();

    for (int j = 0; j < 5; j++) {
      int sensorVal = smoothSensorRead(A1 + j);
      if (sensorVal < minVal[j]) minVal[j] = sensorVal;
      if (sensorVal > maxVal[j]) maxVal[j] = sensorVal;
    }

    // Print raw sensor values during calibration
    Serial.print("Raw Sensor Values: ");
    for (int j = 0; j < 5; j++) {
      Serial.print(analogRead(A1 + j));
      Serial.print(" ");
    }
    Serial.println();
  }

  // Set thresholds based on min/max readings
  for (int i = 0; i < 5; i++) {
    thres[i] = (minVal[i] + maxVal[i]) / 2;
    Serial.print("Threshold for Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(thres[i]);
  }

  motorStop();
  Serial.println("Calibration Complete");
}

// Line following loop with intersection and shape detection
void lineFollowLoop() {
  int sensorVals[5] = {
    smoothSensorRead(Ir1),
    smoothSensorRead(Ir2),
    smoothSensorRead(Ir3),
    smoothSensorRead(Ir4),
    smoothSensorRead(Ir5)
  };

  // Print all sensor values
  Serial.print("Sensors: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(sensorVals[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Check for intersection (when both leftmost and rightmost sensors detect a line)
  if (sensorVals[0] < thres[0] && sensorVals[4] < thres[4]) {
    Serial.println("Intersection detected, moving straight...");
    motorForward(Nor_spR, Nor_spL); // Move straight for a bit
    delay(500);  // Adjust time to fit the length of intersection

    // Check if there's a line in front (using center sensor)
    if (sensorVals[2] < thres[2]) {
      Serial.println("Line detected ahead, returning to normal behavior...");
      linefollow(sensorVals);  // Resume normal line-following behavior
    } 
    else {
      Serial.println("No line ahead, entering shape condition...");
      motorLeftTurn();
      shapeCondition();
    }
  } 
  else {
    // Regular line-following logic
    if (sensorVals[0] < thres[0] && sensorVals[4] > thres[4]) {
      motorRightTurn();
    } 
    else if (sensorVals[4] < thres[4] && sensorVals[0] > thres[0]) {
      motorLeftTurn();
    } 
    else if (sensorVals[2] < thres[2]) {
      linefollow(sensorVals);
    }
  }
}

// Shape condition logic
void shapeCondition() {
  while (true) {  // Stay in this loop until left line is detected
    int sensorVals[5] = {
      smoothSensorRead(Ir1),
      smoothSensorRead(Ir2),
      smoothSensorRead(Ir3),
      smoothSensorRead(Ir4),
      smoothSensorRead(Ir5)
    };

    // Print sensor values during shape condition
    Serial.print("Shape Condition - Sensors: ");
    for (int i = 0; i < 5; i++) {
      Serial.print(sensorVals[i]);
      Serial.print(" ");
    }
    Serial.println();

    // Check for left line detection to exit shape condition
    if (sensorVals[0] < thres[0]) {
      Serial.println("Left line detected, exiting shape condition...");
      motorLeftTurn();
      delay(500);  // Adjust turn timing
      break;  // Exit the shape condition
    }
    
    // Ignore right line detection, but perform normal line-following otherwise
    if (sensorVals[2] < thres[2]) {
      linefollow(sensorVals);  // Normal line following (ignores right line)
    } 
    else if (sensorVals[0] < thres[0]) {
      motorLeftTurn();  // Only react to left line during shape condition
    }
    // Ignore right line (sensor 5), no action taken
  }
}

// Motor control functions for TB6612FNG
void motorForward(int spdR, int spdL) {
  digitalWrite(STBY, HIGH);

  // Right motor forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, spdR);

  // Left motor forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, spdL);
}

void motorRightTurn() {
  digitalWrite(STBY, HIGH);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, Nor_spR - 120);
  analogWrite(ENB, Nor_spL + 60);

  Serial.println("Turning Right");
  delay(30);
}

void motorLeftTurn() {
  digitalWrite(STBY, HIGH);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, Nor_spR + 60);
  analogWrite(ENB, Nor_spL - 120);

  Serial.println("Turning Left");
  delay(30);
}

void motorPendulumRight() {
  digitalWrite(STBY, HIGH);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 70);
  analogWrite(ENB, 70);

  delay(300);
}

void motorPendulumLeft() {
  digitalWrite(STBY, HIGH);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 70);
  analogWrite(ENB, 70);

  delay(300);
}

void motorStop() {
  digitalWrite(STBY, LOW);  // Disable motor controller
}

void linefollow(int sensorVals[]) {
  P = (sensorVals[1] * 0.05 - sensorVals[3] * 0.05);
  I = I + P;
  D = P - pre_P;
  PID = (Kp * P) + (Ki * I) + (Kd * D);
  pre_P = P;

  sp_Rm = Nor_spR + PID;
  sp_Lm = Nor_spL - PID;

  motorForward(sp_Rm, sp_Lm);  // Move forward with adjusted speeds
}
