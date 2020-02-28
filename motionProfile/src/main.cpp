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

void motionProfile() {
  double tSetPoint[17] = {0, 0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2, 2.25, 2.5, 2.75, 3, 3.25, 3.5, 3.75, 4};
  double vSetPoint[17] = {0, 35, 70, 105, 140, 175, 210, 245, 280, 315, 350, 385, 420, 455, 490, 525, 560};
  int lastElement = sizeof(tSetPoint) / sizeof(tSetPoint[0]) - 1;
  Brain.Screen.printAt(1, 120, "%d", lastElement);
  int i = 0;
  myTimer.clear();
  myTimer.reset();
  int currentTime = myTimer.value();
  while(1) {
    currentTime = myTimer.value();
    DriveBase.spin(forward, vSetPoint[i], dps);
    if(currentTime > tSetPoint[i]) {
      i += 1;
    }
    Brain.Screen.printAt(1, 40, "%f", myTimer.value());
    Brain.Screen.printAt(1, 80, "%f", DriveBase.velocity(dps));
    if(currentTime > tSetPoint[lastElement]) {
      DriveBase.stop(coast);
      return;
    }
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  motionProfile();
}
