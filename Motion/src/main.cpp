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

motor_group Left(L1, L2);
motor_group Right(R1, R2);
motor_group DriveBase(L1, L2, R1, R2);

const double TOLERANCE = 2;
const double SET_POINT = 500;
const double MAX_VELOCITY = 80;
const double MIN_VELOCITY = 3;

double constrain (double constrainedValue, double maxValue, double minValue) {
  if(constrainedValue > maxValue) {
    return maxValue;
  }
  if(constrainedValue < -maxValue) {
    return -maxValue;
  }
  if(constrainedValue < minValue && constrainedValue >= 0) {
    return minValue;
  }
  if(constrainedValue > -minValue && constrainedValue <= 0) {
    return -minValue;
  }
  return constrainedValue;
}

double pConstant(double endPoint, double currentRotation) {
  double subPoint = endPoint / 5;
  if (currentRotation < subPoint) {
    return 2;
  } 
  else if (currentRotation < (subPoint * 2)) {
    return 6;
  }
  else if (currentRotation < (subPoint * 3)) {
    return 9;
  }
  else if (currentRotation < (subPoint * 4)) {
    return 11;
  }
  return 12;
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
  double motorRotation = DriveBase.rotation(deg);
  double motorVelocity = 1;
  double kP = 5;
  //Accelerate until mid point k
  while (motorRotation < SET_POINT) {
    motorRotation = DriveBase.rotation(deg);
    double error = SET_POINT - motorRotation;
    motorVelocity = motorRotation / kP;

    motorVelocity = constrain(motorVelocity, MAX_VELOCITY, MIN_VELOCITY);
    DriveBase.spin(forward, motorVelocity, pct);
    Brain.Screen.printAt(1, 40, "%f", motorVelocity);
    Brain.Screen.printAt(1, 80, "%f", motorRotation);

    if(error < TOLERANCE && error > -TOLERANCE) {
      DriveBase.stop(coast);
      return;
    }
  }
  while(1) {
    motorRotation = DriveBase.rotation(deg);
    Brain.Screen.printAt(1, 80, "%f", motorRotation);
  }

  /*
  //Deccelerate until end point
  while (motorRotation < endPoint) {
    double error = endPoint - motorRotation;
    motorVelocity = error * kP;
    motorVelocity = constrain(motorVelocity, maxVelocity, -maxVelocity);
    DriveBase.spin(forward, motorVelocity, percent);
  }
  */
  return;
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  motionProfile();
}
