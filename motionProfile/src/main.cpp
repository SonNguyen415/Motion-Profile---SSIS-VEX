/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       imperium                                                  */
/*    Created:      Tue Jan 07 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// L1                   motor         1               
// L2                   motor         11              
// R1                   motor         2               
// R2                   motor         12              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

motor_group Left = motor_group(L1, L2);
motor_group Right = motor_group(R1, R2);
motor_group DriveBase = motor_group(L1, L2, R1, R2);
timer myTimer = timer();

const double TOLERANCE = 2;
const double SET_POINT = 500;
const double MAX_VELOCITY = 80;
const double MIN_VELOCITY = 3;

double constrain (double constrainedValue, double maxValue, double minValue) {
  if(constrainedValue > maxValue) {
    return maxValue;
  }
  if(constrainedValue < minValue) {
    return minValue;
  }
  return constrainedValue;
}

void normalPID() {
  double motorRotation = DriveBase.rotation(deg);
  double motorVelocity = 1;
  double kP = 5;
  while (1) {
    motorRotation = DriveBase.rotation(deg);
    double error = SET_POINT - motorRotation;
    motorVelocity = error / kP;

    motorVelocity = constrain(motorVelocity, MAX_VELOCITY, MIN_VELOCITY);
    DriveBase.spin(forward, motorVelocity, pct);
    Brain.Screen.printAt(1, 40, "%f", motorVelocity);
    Brain.Screen.printAt(1, 80, "%f", motorRotation);

    if(error < TOLERANCE) {
      DriveBase.stop(coast);
      return;
    }
  }
  while(1) {
    motorRotation = DriveBase.rotation(deg);
    Brain.Screen.printAt(1, 80, "%f", motorRotation);
  } 
}

void motionProfile() {
  double tSetPoint[11] = {0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5};
  double vSetPoint[11] = {0, 14, 28, 42, 56, 70, 84, 98, 112, 126, 140};
  int i = 0;
  myTimer.clear();
  while(1) {
    int currentTime = myTimer.time(timeUnits::sec);
    DriveBase.spin(forward, vSetPoint[i], dps);
    if(currentTime > tSetPoint[i]) {
      i += 1;
    }
    Brain.Screen.printAt(1, 40, "%f", DriveBase.velocity(dps));
  }
  return;
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  motionProfile();
}
