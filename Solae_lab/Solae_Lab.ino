#include <Servo.h> 

Servo horizontal; // Horizontal Servo Motor
int servohori = 180; 
int servohoriLimitHigh = 175;
int servohoriLimitLow = 5;

Servo vertical; // Vertical Servo Motor
int servovert = 45; 
int servovertLimitHigh = 100;
int servovertLimitLow = 1;

// LDR pin connections
int ldrlt = A0; // Bottom Left LDR
int ldrrt = A3; // Bottom Right LDR
int ldrld = A1; // Top Left LDR
int ldrrd = A2; // Top Right LDR

const int currentSensorPin = A4;
const int voltagePin = A5;

unsigned long lastSensorReadTime = 0;
const unsigned long sensorReadInterval = 2000; // Every 2 seconds

float current = 0;
float voltage = 0;
float power = 0;

const int switchPin = 7;  // D7
bool trackingEnabled = true;



void setup() {
  pinMode(switchPin, INPUT_PULLUP);  // Using internal pull-up
  horizontal.attach(5);
  vertical.attach(6);
  horizontal.write(180);
  vertical.write(45);
  Serial.begin(115200);
  delay(2500);
  //int raw = analogRead(currentSensorPin);
  //Serial.println(raw); // See what it gives with 0A

}


void loop() {
trackingEnabled = digitalRead(switchPin); // HIGH = tracking, LOW = non-tracking

if (trackingEnabled) {
  // ----- TRACKING MODE -----
  int lt = analogRead(ldrlt);
  int rt = analogRead(ldrrt);
  int ld = analogRead(ldrld);
  int rd = analogRead(ldrrd);

  int dtime = 18;
  int tol = 20;

  int avt = (lt + rt) / 2;
  int avd = (ld + rd) / 2;
  int avl = (lt + ld) / 2;
  int avr = (rt + rd) / 2;

  int dvert = avt - avd;
  int dhoriz = avl - avr;

  if (abs(dvert) > tol) {
    if (avt > avd) {
      servovert = ++servovert;
      if (servovert > servovertLimitHigh) servovert = servovertLimitHigh;
    } else {
      servovert = --servovert;
      if (servovert < servovertLimitLow) servovert = servovertLimitLow;
    }
    vertical.write(servovert);
  }

  if (abs(dhoriz) > tol) {
    if (avl > avr) {
      servohori = --servohori;
      if (servohori < servohoriLimitLow) servohori = servohoriLimitLow;
    } else {
      servohori = ++servohori;
      if (servohori > servohoriLimitHigh) servohori = servohoriLimitHigh;
    }
    horizontal.write(servohori);
  }

  delay(dtime);

} else {
  // ----- NON-TRACKING MODE: HOLD AT 90 -----
  horizontal.write(90);
  vertical.write(90);
}

  // --- VOLTAGE & CURRENT SENSOR SECTION (Non-blocking) ---
if (millis() - lastSensorReadTime >= sensorReadInterval) {
  lastSensorReadTime = millis();

  // ----- CURRENT READING -----
  float currentSum = 0;
  const int currentSamples = 20;
  for (int i = 0; i < currentSamples; i++) {
    currentSum += (0.0264 * analogRead(currentSensorPin) - 13.51);
  }
  current = currentSum / currentSamples;
  if (abs(current) < 0.05) current = 0.0;
  //float current = (analogRead(A4) - 518) * 0.0264;


  // ----- VOLTAGE READING -----
  int voltageRaw = analogRead(voltagePin);
  float vOut = (voltageRaw * 5.0) / 1023.0;
  voltage = vOut * 2.0; // Assuming 2:1 voltage divider

  // ----- POWER CALCULATION -----
  power = voltage * current * -1;

  // ----- PRINT RESULTS -----
//  // Serial.print(" | Current: ");
//   Serial.print(current, 3);
//   Serial.print(" A");

  Serial.print(" | Voltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V");

  Serial.print(" | Power: ");
  Serial.print(power, 2);
  Serial.println(" W");
}

}