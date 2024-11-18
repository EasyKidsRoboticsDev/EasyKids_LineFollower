#include "EasyKids_LineFollower.h"
#include <Arduino.h>
#include <Servo.h>

static const int IN1_L = 18;
static const int IN2_L = 19;
static const int EN_L =  3;

static const int IN1_R = 10;
static const int IN2_R = 17;
static const int EN_R = 11;

static const int EDF_PIN =  9;
static const int START_SW =  2;

static const int OUT_LINE = 16;

static enum OutState 
{
  CENTER,
  RIGHT,
  LEFT
} out_state = CENTER;

static const int sensorArr[11] = { A7, 4, 5, 6, 7, 8, 12, 14, 15, 16, A6 };
static bool invertedLine = false;
static Servo edf;

void lineFollowerSetup()
{
  Serial.begin(9600);

  for (short int i = 0; i < 11; i++) 
  {
    pinMode(sensorArr[i], INPUT);
  }

  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(EN_L, OUTPUT);

  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(EN_R, OUTPUT);

  pinMode(START_SW, INPUT_PULLUP);

  // Stop Motors
  analogWrite(EN_L, 0);
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN2_L, HIGH);

  analogWrite(EN_R, 0);
  digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, HIGH);
}

void edfSetup()
{
  edf.attach(EDF_PIN); // Brushless attach
  edf.writeMicroseconds(1000); // 1000-2500 maxspeed for Brushless
}

void edfSpeed(int speed)
{
  edf.writeMicroseconds(map(speed, 0, 100, 1000, 2500)); // Map speed for Brushless
}

void edfStop()
{
  edf.writeMicroseconds(1000); // Stop Brushless
}

void Motor_L(int speed)
{
  speed = map(speed, -100, 100, -255, 255); // Max speed = 255
  if (!speed)
  {
    analogWrite(EN_L, 255);
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, HIGH);
  }
  else if(speed > 0)
  {
    speed = min(speed, 255);
    analogWrite(EN_L, speed);
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  }
  else 
  {
    speed = abs(max(speed, -255));
    analogWrite(EN_L, speed);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
  }
}

void Motor_R(signed int speed)
{
  speed = map(speed, -100, 100, -255, 255); // Max speed = 255
  if (!speed)
  {
    analogWrite(EN_R, 255);
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, HIGH);
  }
  else if(speed > 0)
  {
    speed = min(speed, 255);
    analogWrite(EN_R, speed);
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
  }
  else 
  {
    speed = abs(max(speed, -255));
    analogWrite(EN_R, speed);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
  }
}

void waitForStart()
{
  while (!digitalRead(START_SW)) {}
}

int sw_Start()
{
  return digitalRead(START_SW);
}

void blackLine()
{
  invertedLine = false;
}

void whiteLine()
{
  invertedLine = true;
}

static int checkSensor(int pin)
{
  return (analogRead(pin) > 200) ? 1 : 0;
}

void readSensor()
{
  bool sensorVal = 0;
  while (1)
  {
    for (short int i = 0; i < 11; i++) 
    {
      sensorVal = ((i == 0) || (i == 10)) ? !checkSensor(sensorArr[i]) : !digitalRead(sensorArr[i]);
      Serial.print(sensorVal); // Serial Monitor Output
    }
    Serial.println("");
  }
}

static int readError()
{
  bool sensorVal[11] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  bool onLine = 0;
  int error = 0;

  bool cornerSensors = checkSensor(sensorArr[0]) | checkSensor(sensorArr[10]);
  onLine |= invertedLine ? cornerSensors : !cornerSensors;
  for (short int i = 1; i < 10; i++) 
  {
    sensorVal[i] = invertedLine ? digitalRead(sensorArr[i]) : !digitalRead(sensorArr[i]);
    onLine |= sensorVal[i];
  }

  if (!onLine)
  {
    return OUT_LINE;
  }

  /* Negative left sensor */
  error = sensorVal[4]                   ? -2 : error; // b1
  error = sensorVal[3]                   ? -4 : error; // b3
  error = sensorVal[2]                   ? -6 : error; // b5
  error = sensorVal[1]                   ? -8 : error; // b7
  error = sensorVal[0]                   ? -10 : error; // b9
  error = sensorVal[4] && sensorVal[3]   ? -3 : error; // b1 && b3
  error = sensorVal[3] && sensorVal[2]   ? -5 : error; // b3 && b5
  error = sensorVal[2] && sensorVal[1]   ? -7 : error; // b5 && b7
  error = sensorVal[1] && sensorVal[0]   ? -9 : error; // b7 && b9
  
  /* Positive right sensor */
  error = sensorVal[6]                   ? 2 : error; // b2
  error = sensorVal[7]                   ? 4 : error; // b4 
  error = sensorVal[8]                   ? 6 : error; // b6 
  error = sensorVal[9]                   ? 8 : error; // b8
  error = sensorVal[10]                  ? 10 : error; // b10
  error = sensorVal[6] && sensorVal[7]   ? 3 : error; // b2 && b4
  error = sensorVal[7] && sensorVal[8]   ? 5 : error; // b4 && b6
  error = sensorVal[8] && sensorVal[9]   ? 7 : error; // b6 && b8
  error = sensorVal[9] && sensorVal[10]  ? 9 : error; // b8 && b10
  
  /* Neutral middle sensor */
  error = sensorVal[5]                   ? 0 : error; // b0
  error = sensorVal[5] && sensorVal[4]   ? -1 : error; // b0 && b1
  error = sensorVal[5] && sensorVal[6]   ? 1 : error; // b0 && b2
  
  out_state = (error <= 4) && (error >= -4)   ? CENTER : out_state; 
  out_state = (error >= 5) && (error <= 10)   ? LEFT : out_state;
  out_state = (error <= -5) && (error >= -10) ? RIGHT: out_state;

  return error;
}


void pidLine(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD)
{
  static int error_loop_count = 0;
  static int previous_error = 0;
  static int error_sum = 0;

  int speed_1 = 0;
  int speed_2 = 0;

  // Upscaling the coefficient values
  KP *= 10;
  KI *= 10;
  KD *= 10;

  int current_error = readError();
  if (current_error == OUT_LINE)
  {
    switch (out_state)
    {
      case CENTER:
        speed_1 = MED_SPEED;
        speed_2 = MED_SPEED;
        break;
      case LEFT:
        speed_1 = MAX_SPEED;
        speed_2 =  -MAX_SPEED;
        break;
      case RIGHT:
        speed_1 = -MAX_SPEED;
        speed_2 = MAX_SPEED;
        break;
      default:
        break;
    }
  }
  else
  {
    int output = (KP * current_error) + (KI * error_sum) + (KD * (current_error - previous_error));

    error_loop_count++;
    if (error_loop_count > 350)
    {
      error_loop_count = 0;
      previous_error = current_error;
      error_sum += current_error;
    }

    if (current_error == 0)
    {
      error_sum = 0;
    }

    speed_1 = MED_SPEED + output;
    speed_2 = MED_SPEED - output;
  }

  Motor_L(speed_1);
  Motor_R(speed_2);
  
}

void lineTimer(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD, long timer)
{
  long timeSince = millis();
  while (millis() - timeSince < timer)
  {
    pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
  }
  Motor_L(0);
  Motor_R(0);
}

void lineCross(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD) // 00011111000
{
  int ss3 = invertedLine ? !digitalRead(sensorArr[3]) : digitalRead(sensorArr[3]); // Sensor 3
  int ss4 = invertedLine ? !digitalRead(sensorArr[4]) : digitalRead(sensorArr[4]); // Sensor 4
  int ss5 = invertedLine ? !digitalRead(sensorArr[5]) : digitalRead(sensorArr[5]); // Sensor 5
  int ss6 = invertedLine ? !digitalRead(sensorArr[6]) : digitalRead(sensorArr[6]); // Sensor 6
  int ss7 = invertedLine ? !digitalRead(sensorArr[7]) : digitalRead(sensorArr[7]); // Sensor 7
  while(ss3 || ss4 || ss5 || ss6 || ss7)
  {
    pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
  }

  Motor_L(MED_SPEED);
  Motor_R(MED_SPEED);
  delay(map(MED_SPEED, 0, 100, 100, 0));

  Motor_L(0);
  Motor_R(0);
}

void line90Left(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD) // 00100100000
{
  int ss2 = invertedLine ? !digitalRead(sensorArr[2]) : digitalRead(sensorArr[2]); // Sensor 2
  int ss5 = invertedLine ? !digitalRead(sensorArr[5]) : digitalRead(sensorArr[5]); // Sensor 5
  while(ss2 || ss5)
  {
    pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
  }

  Motor_L(MED_SPEED);
  Motor_R(MED_SPEED);
  delay(map(MED_SPEED, 0, 100, 100, 0));

  Motor_L(0);
  Motor_R(0);
}

void line90Right(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD) // 00000100100
{
  int ss5 = invertedLine ? !digitalRead(sensorArr[5]) : digitalRead(sensorArr[5]); // Sensor 5
  int ss8 = invertedLine ? !digitalRead(sensorArr[8]) : digitalRead(sensorArr[8]); // Sensor 8
  while(ss5 || ss8)
  {
    pidLine(MED_SPEED, MAX_SPEED, KP, KI, KD);
  }

  Motor_L(MED_SPEED);
  Motor_R(MED_SPEED);
  delay(map(MED_SPEED, 0, 100, 100, 0));

  Motor_L(0);
  Motor_R(0);
}

void lineTurnLeft(int MED_SPEED) // Second or third leftest sensor
{
  Motor_R(MED_SPEED);
  Motor_L(-MED_SPEED);
  delay(50);

  int ss2 = invertedLine ? !digitalRead(sensorArr[2]) : digitalRead(sensorArr[2]); // Sensor 2
  int ss3 = invertedLine ? !digitalRead(sensorArr[3]) : digitalRead(sensorArr[3]); // Sensor 3
  while(ss2 || ss3)
  {
    Motor_R(MED_SPEED);
    Motor_L(-MED_SPEED);
  }
  Motor_L(0);
  Motor_R(0);
}

void lineTurnRight(int MED_SPEED) // Second or third rightest sensor
{
  Motor_R(-MED_SPEED);
  Motor_L(MED_SPEED);
  delay(50);
  
  int ss7 = invertedLine ? !digitalRead(sensorArr[7]) : digitalRead(sensorArr[7]); // Sensor 7
  int ss8 = invertedLine ? !digitalRead(sensorArr[8]) : digitalRead(sensorArr[8]); // Sensor 8
  while(ss7 || ss8)
  {
    Motor_R(-MED_SPEED);
      Motor_L(MED_SPEED);
  }
  Motor_L(0);
  Motor_R(0);
}