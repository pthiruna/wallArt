
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver();
Adafruit_PWMServoDriver board2 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver board3 = Adafruit_PWMServoDriver(0x42);

byte buttonStateToggle = HIGH;         // variable for reading the pushbutton status
const int buttonUp = 2;
const int buttonDown = 3;
const int buttonShift = 5;
const int buttonSave = 4;
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates

int buttonUpState = 0;
int buttonDownState = 0;
int buttonShiftState = 0;
int buttonSaveState = 0;

void setPulse(byte motor, int pulse)
{
      Serial.print("pwm  ");Serial.print(motor);Serial.print(" pulse:  ");Serial.println(pulse);
    if (motor < 15)
    {
        board1.setPWM(motor, 0, pulse);
    }
    else if (motor > 14 && motor < 25)
    {
        board2.setPWM(motor-15, 0, pulse);
    }
    else if (motor > 24)
    {
        board3.setPWM(motor-25, 0, pulse);
    }
}

void setup() {
    Serial.begin(9600);
    Serial.println("Begin Calibration");
    pinMode(LED_BUILTIN, OUTPUT);
        board1.begin();
    board1.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates
    board2.begin();
    board2.setPWMFreq(SERVO_FREQ);
    board3.begin();
    board3.setPWMFreq(SERVO_FREQ);
}
  void printPosition(int arr[]) {
    for(int i = 0; i < 35; i++)
    {
       Serial.print(arr[i]);Serial.print(",");
    }
  }

int pos = 160;
int stepUp = 1;
int stepDown = -1;
int motor = 0;
int motorPos[35];

void loop() {
  // read the state of the pushbutton value:
  buttonUpState = digitalRead(buttonUp);
  buttonDownState = digitalRead(buttonDown);
  buttonShiftState = digitalRead(buttonShift);
  buttonSaveState = digitalRead(buttonSave);

  if (buttonUpState == HIGH) {
    digitalWrite(LED_BUILTIN, HIGH);
    pos = pos+stepUp;
    stepUp = stepUp +1;
    Serial.print("Motor  ");Serial.print(motor);Serial.print(" pos:  ");Serial.println(pos);
    setPulse(motor, pos);
    delay(160);
  } else {
     digitalWrite(LED_BUILTIN, LOW);
     stepUp = 1;
  }
 
  if (buttonDownState == HIGH) {
    digitalWrite(LED_BUILTIN, HIGH);
    pos= pos + stepDown;
    stepDown = stepDown - 1;
    Serial.print("Motor  ");Serial.print(motor);Serial.print(" pos:  ");Serial.println(pos);
    setPulse(motor, pos);
    delay(160);
  } else {
     digitalWrite(LED_BUILTIN, LOW);
     stepDown = -1;
  }

  if (buttonShiftState == HIGH) {
    digitalWrite(LED_BUILTIN, HIGH);
    if(motor > 34) {
      motor = 0;
    } else {
      motor = motor + 1;
    }
    Serial.print("Motor selected >>>>  ");Serial.println(motor);
    pos = 160;
    delay(300);
  } 

   if (buttonSaveState == HIGH) {
    digitalWrite(LED_BUILTIN, HIGH);
    motorPos[motor]= pos;
    Serial.print("SETTING Motor  ");Serial.print(motor);Serial.print(" pos:  ");Serial.println(pos);
    Serial.println("--------");
    printPosition(motorPos);
    Serial.println("--------");
    delay(300);
  } 

}

