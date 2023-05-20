#include <Servo.h>

Servo motor1;
Servo motor2;

String zeroPadString(int number, int width) {
  String paddedString = String(number);  // Convert the number to a string
 
  // Add leading zeros if necessary
  while (paddedString.length() < width) {
    paddedString = "0" + paddedString;
  }
 
  return paddedString;
}

int joyPinX = A0;
int joyPinY = A1;
int joyPinA = A2; // Joystick Y-axis pin. Left is 0. Right is 1023.
int joyPinB = A3; // Joystick X-axis pin. Up is 0, Down is 1023.
int joyThreshold = 20; // Joystick deadzone threshold
int buttonPin = 2; // Push button pin
int ledPin = 13; // LED pin

bool motorsArmed = false; // Motors armed flag
bool buttonPressed = false; // Button pressed flag
unsigned long lastDebounceTime = 0; // Time of last button state change
unsigned long debounceDelay = 300; // Debounce delay in milliseconds
unsigned long previousMillis = 0; // Previous time the serial monitor was updated
const unsigned long interval = 1000; // Update interval in milliseconds
const unsigned long MINSPEED = 1000;
const unsigned long MAXSPEED = 2000;
const unsigned long MAXTURNDIFF = 255;

const bool enable1 = true;
const bool enable2 = true;

void setup() {
  motor1.attach(3); // Attach motor1 to pin 3. Green pair.
  motor2.attach(5); // Attach motor2 to pin 5. Blue pair.

  motor1.writeMicroseconds(MINSPEED); // Set motor speeds to stop if motors are not armed
  motor2.writeMicroseconds(MINSPEED);

  pinMode(buttonPin, INPUT_PULLUP); // Set push button as input with internal pull-up resistor
  pinMode(ledPin, OUTPUT); // Set LED as output
  Serial.begin(115200);
  Serial.println("Let's Fly");
  Serial.println();
}

void loop() {

  // Read the potentiometers
  int rawX = analogRead(joyPinX); // Read joystick X-axis
  int rawY = analogRead(joyPinY); // Read joystick Y-axis
  int rawA = analogRead(joyPinA);
  int rawB = analogRead(joyPinB);

  int joyX = rawX;
  int joyY = rawY;

  // // Apply deadzone to joystick readings
  // if (abs(joyX - 512) < joyThreshold) {
  //   joyX = 512;
  // }
  // if (abs(joyY - 512) < joyThreshold) {
  //   joyY = 512;
  // }

  // // Map joystick readings to motor speeds
  // int speed = map(joyY, 512, 1023, MINSPEED, MAXSPEED);

  // // Calculate motor speed differentials for turning
  // int turnDiff = map(joyX, 0, 1023, -MAXTURNDIFF, MAXTURNDIFF);

  // // Apply speed differentials for turning
  // int speed1 = speed - turnDiff;
  // int speed2 = speed + turnDiff;

  // // Constrain motor speeds to only go forward
  // speed1 = constrain(speed1, MINSPEED, MAXSPEED);
  // speed2 = constrain(speed2, MINSPEED, MAXSPEED);

  // Map input to output
  int speed1 = map(joyX, 0, 1023, MINSPEED, MAXSPEED);
  int speed2 = map(joyY, 0, 1023, MINSPEED, MAXSPEED);

  // Set motor speeds if motors are armed
  if (motorsArmed) {
    motor1.writeMicroseconds(speed1);
    motor2.writeMicroseconds(speed2);

  } else {
    motor1.writeMicroseconds(MINSPEED); // Set motor speeds to stop if motors are not armed
    motor2.writeMicroseconds(MINSPEED);
  }

  // Update the armed/safe state.
  unsigned long currentMillis = millis();

  // Check if push button is pressed to arm/disarm the motors
  if (digitalRead(buttonPin) == LOW && !buttonPressed) {
    // Record time of button press
    lastDebounceTime = millis();
    buttonPressed = true;
  }

  // Check if debounce delay has elapsed since last button state change
  if (buttonPressed && millis() - lastDebounceTime > debounceDelay) {
    // Toggle motors armed flag
    motorsArmed = !motorsArmed;
    // Turn on LED if motors are armed, otherwise turn off LED
    digitalWrite(ledPin, motorsArmed ? HIGH : LOW);
    // Reset button pressed flag
    buttonPressed = false; // Dan commented this out. See below.
  }

  // Begin Serial Output section
  // Check if the update interval has elapsed
  if (currentMillis - previousMillis >= interval) {
    // Update the previous time
    previousMillis = currentMillis;
    if (motorsArmed){
      Serial.print("HOT  ");
    } else {
      Serial.print("Safe ");
    }
    // Print static text and variable output to the serial monitor

      Serial.print("rawX: ");
      Serial.print(zeroPadString(rawX, 4));
      Serial.print(" ");

      Serial.print("rawY: ");
      Serial.print(zeroPadString(rawY, 4));
      Serial.print(" ");

      // Serial.print("rawA: ");
      // Serial.print(zeroPadString(rawA, 4));
      // Serial.print(" ");

      // Serial.print("rawB: ");
      // Serial.print(zeroPadString(rawB, 4));
      // Serial.print(" ");

      // Serial.print("joyX: ");
      // Serial.print(zeroPadString(joyX, 4));
      // Serial.print(" ");

      // Serial.print("joyY: ");
      // Serial.print(zeroPadString(joyY, 4));
      // Serial.print(" ");


      Serial.print("motor 1: ");
      Serial.print(speed1);
      Serial.print(" ");
      Serial.print("motor 2: ");
      Serial.print(speed2);
      Serial.println(" ");
  }
}
