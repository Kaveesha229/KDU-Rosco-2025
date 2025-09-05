// Multiplexer select pins
const int s0 = A0;
const int s1 = A1;
const int s2 = A2;
const int s3 = A3;

// Multiplexer signal pin
const int SIG_pin = A4;

// Array to store sensor values
int sensorValues[16];

// Custom read order: 9–16, then 8–1 (channel numbers)
int readOrder[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

void setup() {
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // Read sensors in custom order
  for (int i = 0; i < 16; i++) {
    int channel = readOrder[i];
    selectChannel(channel);
    delay(5);  // Optional: for signal stability
    sensorValues[i] = analogRead(SIG_pin);
  }

  // Print sensor values in requested order
  Serial.print("IR Sensor Values (9→16, 8→1): ");
  for (int i = 0; i < 16; i++) {
    Serial.print(sensorValues[8]);
    Serial.print("\t");
  }
  Serial.println();

  delay(200);
}

void selectChannel(int channel) {
  digitalWrite(s0, bitRead(channel, 0));
  digitalWrite(s1, bitRead(channel, 1));
  digitalWrite(s2, bitRead(channel, 2));
  digitalWrite(s3, bitRead(channel, 3));
}
