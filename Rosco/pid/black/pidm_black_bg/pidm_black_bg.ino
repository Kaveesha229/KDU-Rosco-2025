#include <Wire.h>
#include <NewPing.h>

#define THRESHOLD 850
#define MAX_SPEED 255
#define turn_speed 110

// Motor driver pins
#define AIN1 8  // Motor A Input 1 right
#define AIN2 3  // Motor A Input 2
#define PWMA 5  // Motor A PWM (Speed Control)

#define BIN1 4  // Motor B Input 1
#define BIN2 7  // Motor B Input 2
#define PWMB 6  // Motor B PWM (Speed Control)

#define TRIGGER_PIN_RIGHT 11  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_RIGHT 12     // Arduino pin tied to echo pin on the ultrasonic sensor.

#define TRIGGER_PIN_LEFT 9  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_LEFT 10    // Arduino pin tied to echo pin on the ultrasonic sensor.

#define MAX_DISTANCE 200

int MotorBasespeed = 255;

// Multiplexer select pins
const int s0 = A0;
const int s1 = A1;
const int s2 = A2;
const int s3 = A3;

// Multiplexer signal pin
const int SIG_pin = A4;

#define irr 13
#define irl 2

int irl_value;
int irr_value;

int IR_VAL[16] = { 0 };
int IR_weights[16] = { -85, -65, -50, -35, -25, -15, -10, -5, 85, 65, 50, 35, 25, 15, 10, 5 };

int IR_PINS[16];

// Order of reading multiplexer channels (adjust based on wiring)
//int readOrder[16] = {8, 9, 10, 11, 12, 13, 14, 15, 7, 6, 5, 4, 3, 2, 1, 0};
//int readOrder[16] = {15, 14, 13, 12, 11, 10, 9, 8, 0, 1, 2, 3, 4, 5, 6, 7};
int readOrder[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };


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
void toleft();
void toright();
void turn_right();
void turn_left();
void turn();


NewPing right_sonar(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);   // NewPing setup of pins and maximum distance.
NewPing left_sonar(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);  // NewPing setup of pins and maximum distance.

bool wall_available = 0;
float last_wall_error = 0.0;
float wall_error_sum = 0.0;

int distance_to_left_wall = 0;
int distance_to_right_wall = 0;

void wall_pid(uint8_t minSpeed, uint8_t baseSpeed, uint8_t maxSpeed, float Kwp, float Kwi, float Kwd, bool wall = 0);

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

  delay(1000);
  linefollowing(150);
  delay(1000);
}



void loop() {
  readir();
  linefollowing(110);
   //turn();
 // wall_following();
}



void wall_following() {
  //wall_pid(0, 70, 70, 20, 0, 20);
  //wall_pid(0, 70, 70, 14, 0, 20);
  //wall_pid(0, 70, 70, 10, 0, 0.5);
  //wall_pid(0, 150, 150, 25, 0, 15);
  //wall_pid(0, 150, 150, 17, 0.1, 23);
  wall_pid(0, 255, 255, 17, 0.0, 22);
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
    IR_VAL[i] = (analogValue < THRESHOLD) ? 1 : 0;
  }
}

void set_speed(int leftSpeed, int rightSpeed) {
  analogWrite(PWMA, leftSpeed);   // Left Motor
  analogWrite(PWMB, rightSpeed);  // Right Motor
}

void linefollowing(int baseSpeed) {
  MotorBasespeed = baseSpeed;
  set_forward();  // ✅ Corrected: now properly defined
  read_IR();
  PID_control();
  set_speed(LMOTORSpeed, RMOTORSpeed);
}


void wall_pid (uint8_t minSpeed, uint8_t baseSpeed, uint8_t maxSpeed, float Kwp, float Kwi, float Kwd, bool wall) {  //wall follow 

  int distance_to_left = left_sonar.ping_cm() + 5 ;
  int distance_to_right = right_sonar.ping_cm() + 5;

  float wall_error = (float)distance_to_left - (float)distance_to_right;
  float output = Kwp * wall_error + Kwi * wall_error_sum + Kwd * (wall_error - last_wall_error);

  Serial.print(wall_error);
  Serial.print("   ");
  Serial.print("PID ");
  Serial.print(output);
  Serial.println("");
  //delay(10);
  
  wall_error_sum += wall_error;
  last_wall_error = wall_error;
  
  int LMOTORSpeed = MotorBasespeed - (int)output;
  int RMOTORSpeed = MotorBasespeed + (int)output;

  LMOTORSpeed = constrain(LMOTORSpeed, minSpeed, maxSpeed); 
  RMOTORSpeed = constrain(RMOTORSpeed, minSpeed, maxSpeed);
  
  set_forward();
  set_speed(LMOTORSpeed, RMOTORSpeed);
}

void toleft() {
  irl_value = digitalRead(irl);
  if (irl_value == 0) {

    readir();
    read_IR();  // Initial sensor read
    bool foundline = false;

    unsigned long startTime = millis();
    while (millis() - startTime < 300) {
      linefollowing(110);
    }
    // First check if we're already on a line
    // for (int i = 0; i < 16; i++) {
    //   if (IR_VAL[i] == 0) {
    //     foundline = true;
    //     break;
    //   }
    // }

    if (!foundline) {
      // Center on intersection first
      // unsigned long startTime = millis();
      // while (millis() - startTime < 200) {
      //   linefollowing(110);
      // }

      turn_left();
      delay(250);

      while (IR_VAL[7] == 1) {
        readir();
        read_IR();  // IR index 7 = center sensor
        turn_left();
      }

      set_backward();
      delay(50);
    }
  }
}

void toright() {
  irr_value = digitalRead(irr);
  if (irr_value == 0) {

    readir();
    read_IR();  // Initial sensor read
    bool foundline = false;

    unsigned long startTime = millis();
    while (millis() - startTime < 300) {
      linefollowing(110);
    }

    // First check if we're already on a line
    // for (int i = 0; i < 16; i++) {
    //   if (IR_VAL[i] == 0) {
    //     foundline = true;
    //     break;
    //   }
    // }

    if (!foundline) {
      // unsigned long startTime = millis();
      // while (millis() - startTime < 500) {
      //   linefollowing(110);
      // }

      // turn_right();
      // delay(250);

      while (IR_VAL[7] == 1) {
        readir();
        read_IR();  // IR index 7 = center sensor
        turn_right();
      }

      set_backward();
      delay(50);
    }
  }
}


void turn() {
  toleft();
  toright();
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
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  Serial.println("Moving STOP");
}

void turn_right() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  set_speed(turn_speed, turn_speed);
}

void turn_left() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  set_speed(turn_speed, turn_speed);
}
