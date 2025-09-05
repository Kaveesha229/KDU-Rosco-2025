#define THRESHOLD 950
#define MAX_SPEED 150
#define turn_speed 150

int MotorBasespeed = 255;

// Multiplexer select pins
const int s0 = A0;
const int s1 = A1;
const int s2 = A2;
const int s3 = A3;

// Multiplexer signal pin
const int SIG_pin = A4;

int IR_VAL[16] = {0};
int IR_weights[16] = {-85, -65, -50, -35, -25, -15, -10, -5, 85, 65, 50, 35, 25, 15, 10, 5};

int IR_PINS[16];

// Order of reading multiplexer channels (adjust based on wiring)
//int readOrder[16] = {8, 9, 10, 11, 12, 13, 14, 15, 7, 6, 5, 4, 3, 2, 1, 0};
//int readOrder[16] = {15, 14, 13, 12, 11, 10, 9, 8, 0, 1, 2, 3, 4, 5, 6, 7};
int readOrder[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

// Motor driver pins
#define AIN1  8    // Motor A Input 1  right
#define AIN2  3    // Motor A Input 2  
#define PWMA  5    // Motor A PWM (Speed Control)  

#define BIN1  4    // Motor B Input 1  
#define BIN2  7    // Motor B Input 2  
#define PWMB  6    // Motor B PWM (Speed Control)   

int RMOTORSpeed = 0;
int LMOTORSpeed = 0;
int speedAdjust = 0;

// PID variables
float P, I, D;
float error = 0;
float previousError = 0;
float Kp = 0.8;
float Kd = 0.2;
float Ki = 0;

// Function declarations
void PID_control();
void read_IR();
void set_speed();
void set_forward();  // ✅ Added this function properly
void set_backward();
void linefollowing(int baseSpeed);
void readir();
void selectChannel(int channel);

void setup() {
  // Set motor driver pins as output
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  readir();
  linefollowing(80);
}

void readir() {
  for (int i = 0; i < 16; i++) {
    int channel = readOrder[i];
    selectChannel(channel);
    delayMicroseconds(2);  // Fast multiplexer read
    IR_PINS[i] = analogRead(SIG_pin);
  }

  // Debugging sensor values
  Serial.print("IR Sensor Values (9→16, 8→1): ");
  for (int i = 0; i < 16; i++) {
    Serial.print(IR_VAL[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void selectChannel(int channel) {
  digitalWrite(s0, bitRead(channel, 0));
  digitalWrite(s1, bitRead(channel, 1));
  digitalWrite(s2, bitRead(channel, 2));
  digitalWrite(s3, bitRead(channel, 3));
}

void PID_control() {
  error = 0;
  for (int i = 0; i < 16; i++) {
    error += IR_weights[i] * IR_VAL[i];
  }

  P = error;
  I += error;
  D = error - previousError;
  previousError = error;

  speedAdjust = (Kp * P + Ki * I + Kd * D);

  RMOTORSpeed = MotorBasespeed - speedAdjust + 6;
  LMOTORSpeed = MotorBasespeed + speedAdjust;

  RMOTORSpeed = constrain(RMOTORSpeed, 0, MAX_SPEED);
  LMOTORSpeed = constrain(LMOTORSpeed, 0, MAX_SPEED);
}

void read_IR() {
  for (int i = 0; i < 16; i++) {
    int analogValue = IR_PINS[i];  // Already filled in readir()
    IR_VAL[i] = (analogValue > THRESHOLD) ? 1 : 0;
  }
}

void set_speed() {
  analogWrite(PWMA, LMOTORSpeed);  // Left Motor
  analogWrite(PWMB, RMOTORSpeed);  // Right Motor
}

void set_forward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  Serial.println("Moving Forward");
}

void set_backward() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  Serial.println("Moving Backward");
}

void linefollowing(int baseSpeed) {
  MotorBasespeed = baseSpeed;
  set_forward();        // ✅ Corrected: now properly defined
  read_IR();
  PID_control();
  set_speed();
}
