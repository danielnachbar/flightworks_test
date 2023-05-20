/ Joystick mode - read joystick, calculate "joy" values, added base_speed.
// Added attenuation of speed
// Added turndiff (unattenuated)
// Added turn attenuation
// Swapped X and Y on joystick

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
int JOYTHRESHOLD = 40; // Joystick deadzone threshold
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

const int POTMIN = 0;
const int POTMIDPOINT = 512;
const int POTMAX = 1023;

void setup() {
  motor1.attach(3); // Attach motor1 to pin 3. Green pair. Right motor.
  motor2.attach(5); // Attach motor2 to pin 5. Blue pair. Left motor.

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
  int rawX = analogRead(joyPinX); // Read near dial.
  int rawY = analogRead(joyPinY); // Read far dial.
  int rawA = analogRead(joyPinA); // Read joystick X-axis
  int rawB = analogRead(joyPinB); // Read joystick Y-axis

  int speed1 = MINSPEED;
  int speed2 = MINSPEED;

  // Create "joy" abstractions.
  int joyX = -1; // Intitially set to out of band value
  int joyY = -1;
  int invertedY = -1;
  int limitedY = 1;

  // For X
  //   invert the slope
  //   make POTMIDPOINT the center
  joyX = map(rawA, 0, 1023, 1023, 0);  // invert the slope
  joyX = joyX - POTMIDPOINT; // make the midpoint zero
  if (abs(joyX) < JOYTHRESHOLD) { // Add deadzone
    joyX = 0;
  }

  // For Y -
  //   invert the slope
  //   limit to only forward
  //   Shift down so that midpoint is zero
  //   add a deadzone above the midpoint
  invertedY = map(rawB, 0, 1023, 1023, 0);
  if (invertedY < POTMIDPOINT + JOYTHRESHOLD) {
    limitedY = POTMIDPOINT;
  } else {
    limitedY = invertedY;
  }
  joyY = limitedY - POTMIDPOINT; //

  float speed_attenuation = (1023.0-rawY)/POTMAX;
  int attenuated_joyY = (int) joyY * speed_attenuation;

  // // Map joystick, which ranges from 0 to POTMIDPOINT to base motor speed
  int base_speed = map(attenuated_joyY, 0, POTMIDPOINT, MINSPEED, MAXSPEED);

  // // Calculate motor speed differentials for turning
  int base_turndiff = map(joyX, -POTMIDPOINT, POTMIDPOINT, -MAXTURNDIFF, MAXTURNDIFF);

  float turn_attenuation = (1023.0-rawX)/POTMAX;

  int attenuated_turndiff = (int) base_turndiff * turn_attenuation;

  // Apply speed differentials for turning
  speed1 = base_speed - attenuated_turndiff;
  speed2 = base_speed + attenuated_turndiff;

  // Constrain motor speeds to only go forward
  speed1 = constrain(speed1, MINSPEED, MAXSPEED);
  speed2 = constrain(speed2, MINSPEED, MAXSPEED);

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

      Serial.print("A: ");
      Serial.print(zeroPadString(rawA, 4));
      Serial.print(" ");

      Serial.print("B: ");
      Serial.print(zeroPadString(rawB, 4));
      Serial.print(" ");

      Serial.print("X: ");
      Serial.print(zeroPadString(rawX, 4));
      Serial.print(" ");

      Serial.print("Y: ");
      Serial.print(zeroPadString(rawY, 4));
      Serial.print(" ");

      Serial.print("jX: ");
      Serial.print(zeroPadString(joyX, 4));
      Serial.print(" ");

      Serial.print("jY: ");
      Serial.print(zeroPadString(joyY, 4));
      Serial.print(" ");

      Serial.print("s_att: ");
      Serial.print(speed_attenuation);
      Serial.print(" ");

      Serial.print("att_joyY: ");
      Serial.print(zeroPadString(attenuated_joyY, 4));
      Serial.print(" ");

      Serial.print("base_s: ");
      Serial.print(base_speed);
      Serial.print(" ");

      Serial.print("turn_att: ");
      Serial.print(turn_attenuation);
      Serial.print(" ");

      Serial.print("base_td: ");
      Serial.print(base_turndiff);
      Serial.print(" ");

      Serial.print("att_td: ");
      Serial.print(attenuated_turndiff);
      Serial.print(" ");

      Serial.print("motor 1: ");
      Serial.print(speed1);
      Serial.print(" ");
      Serial.print("motor 2: ");
      Serial.print(speed2);
      Serial.println(" ");
  }
}
