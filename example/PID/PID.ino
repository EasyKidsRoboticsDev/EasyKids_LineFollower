#include <EasyKids_LineFollower_V2.h>  /* Select robot version */
// #include <EasyKids_LineFollower.h>

/*
<<<<< Command >>>>>

  lineFollowerSetup();

  blackLine();
  whiteLine();

  readSensor();

  Motor_L(Speed); (-100 to 100)
  Motor_R(Speed); (-100 to 100)

  edfSetup(); 
  edfSpeed(speed); Speed (1-100) // Suction Motor
  edfStop();

  pidLine(Speed, Max_Speed, KP, KD);
  lineTimer(Speed, Max_Speed, KP, KD, Time(ms)); // PID + Timer
  lineCross(Speed, Max_Speed, KP, KD); // PID Until Cross
  line90Left(Speed, Max_Speed, KP, KD);  // PID Until Line on Left
  line90Right(Speed, Max_Speed, KP, KD); // PID Until Line on Right
  lineTurnLeft(Speed); // Turn Left
  lineTurnRight(Speed); // Turn Right
   
*/

void setup() {
  lineFollowerSetup();
  edfSetup(); 
  blackLine();
}

void loop() {

  // readSensor();  // Show Value Sensor via LCD Display

  waitForStart();

  //  ------ Start EDF -------
  edfSpeed(13); // Starting speed
  delay(2000); // delay to start EDF
  
  // ------ Start Robot -------
  lineTimer(20, 100, 2, 0.00001, 1, 3000); 
  lineCross(30, 100, 2, 0.00001, 1); 
  lineTurnRight(30); 
  line90Left(30, 100, 2, 0.00001, 1); 

  // Stop EDF Controller
  edfStop();
  delay(2000);
 
}


