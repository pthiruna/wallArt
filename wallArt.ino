#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver();
Adafruit_PWMServoDriver board2 = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver board3 = Adafruit_PWMServoDriver(0x42);

#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates
#define SERVO_COUNT 35
#define MIN 150
#define MAX 640


float servoConf[SERVO_COUNT][2] = {
    {MIN + 5, MAX}, // 0
    {MIN - 9, MAX + 4}, // 1
    {MIN - 15, MAX - 34}, // 2
    {MIN - 12, MAX - 28}, // 3
    {MIN - 14, MAX - 28}, // 4
    
    {MIN - 12, MAX -22}, // 5
    {MIN - 18, MAX - 22}, //6
    {MIN - 22, MAX - 36}, //7
    {MIN - 26, MAX -54}, //8
    {MIN - 8, MAX -26}, //9
    
    {MIN + 12, MAX +10}, //10
    {MIN + 8, MAX + 10}, //11
    {MIN + 10, MAX +10}, //12
    {MIN - 4, MAX - 6}, //13
    {MIN - 4, MAX - 8}, //14
    
    //4
    {MIN, MAX -2}, //15
    {MIN, MAX +18}, //16
    {MIN + 10, MAX+ 20}, //17
    {MIN + 10, MAX + 32}, //18
    {MIN - 2, MAX +4 }, //19
   
    //5
    {MIN - 16, MAX - 14}, // 20
    {MIN + 2, MAX + 14}, // 21
    {MIN -2, MAX}, // 22
    {MIN + 6, MAX + 34}, // 23
    {MIN - 2, MAX + 2}, // 24
    
    //6
    {MIN - 4, MAX + 24}, // 25
    {MIN - 18, MAX - 22}, //26
    {MIN , MAX - 6 }, //27
    {MIN -2, MAX +8}, //28
    {MIN -2, MAX +2}, //29
    
    //7
    {MIN +8, MAX +4 }, // 30
    {MIN - 16, MAX -28}, // 31
    {MIN, MAX - 6}, // 32
    {MIN, MAX -3}, // 33
    {MIN -6, MAX -26} // 34
};

uint16_t getAdjAngle(uint8_t servo, byte angle)
{
    float unit = 0.00;
    unit = (servoConf[servo][1] - servoConf[servo][0]) / 180;
//    Serial.print("motor "); Serial.print(servo);Serial.print(" angle ");Serial.println(angle);
//    Serial.print("unit"); Serial.println(unit);
    uint16_t result = (angle * unit) + servoConf[servo][0];
//    Serial.println(result);
    return result;
}

byte matrix [7][5] = {
    {30, 31, 32, 33, 34},
    {25, 26, 27, 28, 29},
    {20, 21, 22, 23, 24},
    {15, 16, 17, 18, 19},
    {10, 11, 12, 13, 14},
    {5,   6,  7,  8, 9 },
    {0,   1,  2,  3, 4 },
};

byte curPos[35] = {
               0, 0 ,0 ,0 ,0,
               0, 0 ,0 ,0 ,0,
               0, 0 ,0 ,0 ,0,
               0, 0 ,0 ,0 ,0,
               0, 0 ,0 ,0 ,0,
               0, 0 ,0 ,0 ,0,
               0, 0 ,0 ,0 ,0
};
byte targetPos[35];

//  Reference Table
//      #0  #1  #2  #3  #4
//
// #0  {30, 31, 32, 33, 34}
// #1  {25, 26, 27, 28, 29},
// #2  {20, 21, 22, 23, 24},
// #3  {15, 16, 17, 18, 19},
// #4  {10, 11, 12, 13, 14},
// #5  {5,   6,  7,  8, 9 },
// #6  {0,   1,  2,  3, 4 },

void allMotorsToAngle(byte angle ) {
    for(int i = 0; i < SERVO_COUNT; i++)
    {
      setPWM(i, angle);
    }
}

// diag = 0 to  10
void setDiagonalToAngle(uint8_t diag, uint8_t angle) {
    uint8_t motors = diag >3 && diag <7 ? 4 : (diag > 6 ? 11 - diag : diag);
    uint8_t curMotor = diag > 4 ? (diag-4)*5 + 4: diag;
//    Serial.print("total");Serial.println(motors);
    for (uint8_t i =0 ; i <=motors; i++) {
//         Serial.print("curMotor");Serial.println(curMotor);
         targetPos[curMotor]  = angle;
         curMotor = curMotor + 4;
    }
}
void printPosition(byte arr[]) {
  for(int i = 0; i < SERVO_COUNT; i++)
  {
    Serial.print(" "); Serial.print(arr[i]);Serial.print(" ");
  }
  Serial.println(" ");
}


void bounce (byte steps, uint16_t pause, byte min, byte max, byte count) {
      while(count != 0 ) {
        for (byte motor = 0; motor < SERVO_COUNT; motor++)
        {
            int curSteps = steps;
            if (curPos[motor] == targetPos[motor]) {
                if (motor == 0 && curPos[motor] == min) {
                  count--;
                }
                if (curPos[motor] == min) {
                  targetPos[motor] = max;
                } else {
                  targetPos[motor] = min;
                }
                continue;
            }
            if (curPos[motor] > targetPos[motor]) {
                curSteps = -steps;
            }
            byte nextPos;
            if (abs(targetPos[motor]-curPos[motor]) < abs(curSteps)) {
                nextPos = targetPos[motor];
            } else {
                nextPos = (byte) (curPos[motor] + curSteps );
            }
            setPWM(motor, nextPos);
        }
//        printPosition(curPos);
        delay(pause);
      }
}
void goToTargetPosition(byte steps, uint16_t pause) {
    boolean targetReached = false;
    while(!targetReached) {
        targetReached = true;
        for (byte motor = 0; motor < SERVO_COUNT; motor++)
        {
            int curSteps = steps;
            if (curPos[motor] == targetPos[motor]) {
                continue;
            }
            targetReached = false;
            if (curPos[motor] > targetPos[motor]) {
                curSteps = -steps;
            }
            byte nextPos;
            if (abs(targetPos[motor]-curPos[motor]) < abs(curSteps)) {
                nextPos = targetPos[motor];
            } else {
                nextPos = (byte) (curPos[motor] + curSteps );
            }
            setPWM(motor, nextPos);
        }
//        printPosition(curPos);
        delay(pause);
    }
}
void motorToAngle(byte motor, byte targetAngle, int steps, uint16_t delay) {
    if (curPos[motor] > targetAngle) {
        steps = -steps;
    }
    while (curPos[motor] != targetAngle) {
        byte nextPos;
        if (abs(targetAngle-curPos[motor]) < abs(steps)) {
            nextPos = targetAngle;
        } else {
            nextPos = (byte) (curPos[motor] + steps );
        }
        setPWM(motor, nextPos);
    }

}


void setRowsToAngle(uint8_t rows[], byte rowsCount, byte angle) {
  for(byte i = 0; i <rowsCount; i++) {
    setRowToAngle(rows[i], angle);
  }
}

void setRowToAngle(byte row, byte angle) {
  for (byte i =0; i < 5; i ++) {
    targetPos[i+(row*5)]  = angle;
  }
}

void colToAngleRev(byte col, byte angle, boolean, uint16_t delay) {
    colRangeToAngle(col, 6, 0, angle, delay);
}

void colToAngle(byte col, byte angle, boolean, uint16_t delay) {
    colRangeToAngle(col, 0, 6, angle, delay);
}
void colRangeToAngle(byte col, byte rowBegin, byte rowEnd,  byte angle, uint16_t pause) {
    if(rowBegin < rowEnd) {
        for(byte i=rowBegin; i<= rowEnd; i++ ) {
            setPWM(matrix[i][col], angle);
            if (delay > 0) {
                delay(pause);
            }
        }
    } else {
        for(byte i=rowEnd; i>= rowBegin; i-- ) {
            setPWM(matrix[i][col], angle);
            if (delay > 0) {
                delay(pause);
            }
        }
    }
}


void setup()
{
    Serial.begin(9600);
    Serial.println("8 channel Servo test!");
    //  pwm.setOscillatorFrequency(27000000);

    board1.begin();
    board1.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates
    board2.begin();
    board2.setPWMFreq(SERVO_FREQ);
    board3.begin();
    board3.setPWMFreq(SERVO_FREQ);
    setInitAngle();
}

void setPWM(byte motor, byte angle)
{
    if (motor < 15)
    {
        board1.setPWM(motor, 0, getAdjAngle(motor, angle));
    }
    else if (motor > 14 && motor < 25)
    {
        board2.setPWM(motor-15, 0, getAdjAngle(motor, angle));
    }
    else if (motor > 24)
    {
        board3.setPWM(motor-25, 0, getAdjAngle(motor, angle));
    }
    curPos[motor] = angle;
}

void setInitAngle()
{
    updateTargetPos(84);
    goToTargetPosition(3,10);
}

void updateTargetPos(int inp[35]) {
  for(int i = 0; i < SERVO_COUNT; i++)
  {
    targetPos[i] = inp[i];
  }
}
void updateTargetPos(byte angle) {
  for(int i = 0; i < SERVO_COUNT; i++)
  {
    targetPos[i] = angle;
  }
}

void diagonalWave() {
      updateTargetPos(45);
    goToTargetPosition(3,10);

    for( uint8_t i =0; i < 11 ; i++ ) {
       setDiagonalToAngle(i, 135);
       goToTargetPosition(3, 1);
       printPosition(curPos);
//       delay(10);
    }
    for( uint8_t i =0; i < 11 ; i++ ) {
       setDiagonalToAngle(i, 110);
       goToTargetPosition(3, 1);
       printPosition(curPos);
//       delay(10);
    }
        for( uint8_t i =0; i < 11 ; i++ ) {
       setDiagonalToAngle(i, 75);
       goToTargetPosition(3, 1);
       printPosition(curPos);
//       delay(10);
    }

                for( uint8_t i =0; i < 11 ; i++ ) {
       setDiagonalToAngle(i, 45);
       goToTargetPosition(3, 1);
       printPosition(curPos);
//       delay(10);
    }
                    for( uint8_t i =0; i < 11 ; i++ ) {
       setDiagonalToAngle(i, 30);
       goToTargetPosition(3, 1);
       printPosition(curPos);
//       delay(10);
    }
}
void loop()
{
  diagonalWave();

//    Serial.println(">>>>>>>>boo boo butt >>>>>>>>>>>>>>>>");
     // Wave left right
     uint8_t rows[] = {0,2,4,6,1,3,5};
     setRowsToAngle(rows, 7, 45);
     goToTargetPosition(3,10);
     bounce(2, 0, 45, 135, 5);

     // break your hips
     uint8_t rowsEven[] = {0,2,4,6};
     uint8_t rowsOdd[] = {1,3,5};
     setRowsToAngle(rowsEven, 4, 45);
     setRowsToAngle(rowsOdd, 3, 135);
     goToTargetPosition(3,10);
     bounce(2, 0, 45, 135, 5);

     // topple
     setRowsToAngle(rowsEven, 4, 0);
     setRowsToAngle(rowsOdd, 3, 85);
     goToTargetPosition(3,10);
     bounce(2, 0, 0, 180, 5);

     // Wind
     byte fr[5] = {30, 45, 60, 75, 90};
     int tarPos[35] = {
             fr[0], fr[1], fr[2], fr[3], fr[4],
             fr[0]-5, fr[1]-5, fr[2]-5, fr[3]-5, fr[4]-5,
             fr[0] -10, fr[1] -10, fr[2]-10, fr[3]-10, fr[4]-10,
             fr[0]-15, fr[1]-15, fr[2]-15, fr[3]-15, fr[4]-15,
             fr[0]-20, fr[1]-20, fr[2]-20, fr[3]-20, fr[4]-20,
             fr[0]-25, fr[1]-25, fr[2]-25, fr[3]-25, fr[4]-25,
             fr[0]-30, fr[1]-30, fr[2]-30, fr[3]-30, fr[4]-30
             };
     updateTargetPos(tarPos);
     goToTargetPosition(3, 10);
     updateTargetPos(160);
     bounce(2, 0, 20, 160, 10);

}




//void rowToAngleRev(byte row, byte angle, uint16_t pause) {
//    rowRangeToAngle(row, 4, 0, angle, pause);
//}
//
//void rowToAngle(byte row, byte angle, uint16_t pause) {
//    rowRangeToAngle(row, 0, 4, angle, pause);
//}
//void rowRangeToAngle(byte row, byte colBegin, byte colEnd, byte angle, uint16_t pause) {
//    if (colBegin < colEnd) {
//        for(byte i=colBegin; i<= colEnd;i++) {
//            setPWM(matrix[row][i], angle);
//            if (delay > 0) {
//                delay(pause);
//            }
//        }
//    } else {
//        for(byte i=colEnd; i >= colEnd;i--) {
//            setPWM(matrix[row][i], angle);
//            if (delay > 0) {
//                delay(pause);
//            }
//        }
//    }
//}
