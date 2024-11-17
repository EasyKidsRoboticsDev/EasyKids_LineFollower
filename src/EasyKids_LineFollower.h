#ifndef EasyKids_LineFollower_h
#define EasyKids_LineFollower_h

// Restrict header files to be included
#define USING_V1
#ifdef USING_V2
#error "Cannot use both versions at the same time. Please select only one version"
#endif

void lineFollowerSetup();

void edfSetup();
void edfSpeed(int speed);
void edfStop();

void Motor_L(int speed);
void Motor_R(int speed);
 
void waitForStart();
int sw_Start();

void blackLine();
void whiteLine();

void readSensor();
void pidLine(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD);
void lineTimer(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD, long timer);
void lineCross(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD); 
void line90Left(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD); 
void line90Right(int MED_SPEED, int MAX_SPEED, float KP, float KI, float KD); 
void lineTurnLeft(int MED_SPEED); 
void lineTurnRight(int MED_SPEED); 

#endif // EasyKids_LineFollower.h