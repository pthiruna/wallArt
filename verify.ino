#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver();
Adafruit_PWMServoDriver board2 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver board3 = Adafruit_PWMServoDriver(0x42);
#define SERVO_COUNT 35

int servoConf[SERVO_COUNT][2] = {
{ 140 , 623 },
{ 126 , 627 },
{ 149 , 624 },
{ 127 , 591 },
{ 144 , 635 },
{ 143 , 629 },
{ 145 , 625 },
{ 125 , 592 },
{ 124 , 582 },
{ 124 , 584 },//
    
{ 153 , 623 },
{ 133 , 616 },
{ 137 , 610 },
{ 124 , 589 },
{ 129 , 595 },
{ 135 , 625 },
{ 134 , 630 },
{ 130 , 610 },
{ 130 , 616 },    // tbd
{ 158 , 661 },
    
{ 129 , 628 },
{ 143 , 653 },
{ 129 , 625 },
{ 152 , 672 },
{ 158 , 664 },
{ 130 , 610 },
{ 135 , 632 },
{ 128 , 605 },
{ 144 , 638 },
{ 126 , 612 },
    
{ 133 , 603 },
{ 126 , 599 },
{ 141 , 625 },
{ 115 , 572 },
{ 115 , 566 }
};

byte buttonStateToggle = HIGH;         // variable for reading the pushbutton status
const int buttonUp = 2;
const int buttonDown = 3;
const int buttonShiftUp = 5;
const int buttonShiftDwn = 4;
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates

int buttonUpState = 0;
int buttonDownState = 0;
int buttonShiftMotorUpState = 0;
int buttonShiftMotorDwnState = 0;

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
  buttonUpState = digitalRead(buttonDown);
  buttonDownState = digitalRead(buttonUp);
  buttonShiftMotorUpState = digitalRead(buttonShiftUp);
  buttonShiftMotorDwnState = digitalRead(buttonShiftDwn);

  if (buttonUpState == HIGH) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("Motor  ");Serial.print(motor);Serial.print(" pos:  ");Serial.println(servoConf[motor][1]);
    setPulse(motor, servoConf[motor][1]);
    delay(500);
  } else {
     digitalWrite(LED_BUILTIN, LOW);
  }
 
  if (buttonDownState == HIGH) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("Motor  ");Serial.print(motor);Serial.print(" pos:  ");Serial.println(servoConf[motor][0]);
    setPulse(motor, servoConf[motor][0]);
    delay(500);
  } else {
     digitalWrite(LED_BUILTIN, LOW);
     stepDown = -1;
  }

  if (buttonShiftMotorUpState == HIGH) {
    digitalWrite(LED_BUILTIN, HIGH);
    if(motor == 34) {
      motor = 0;
    } else {
      motor = motor + 1;
    }
    Serial.print("Motor selected >>>>  ");Serial.println(motor);
    delay(300);
  } 
  if (buttonShiftMotorDwnState == HIGH) {
    digitalWrite(LED_BUILTIN, HIGH);
    if(motor == 0) {
      motor = 34;
    } else {
      motor = motor - 1;
    }
    Serial.print("Motor selected >>>>  ");Serial.println(motor);
    delay(300);
  } 

}

