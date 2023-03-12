/*
    
    motorSetup.h
    Purpose: Used to initalize all motors, sensors, controllers, and classes used on the robot

*/


#include "odom.h"
#include "main.h"
#include "pid.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include <vector>

using namespace pros;

const double ticksPerDeg = 900 / 360;
const double robotSpeed = 200;
const double rotationSpeed = 200;

extern pros::Motor FrontLeft;
extern pros::Motor FrontRight;
extern pros::Motor BackLeft;
extern pros::Motor BackRight;

// Ports
const int FLPort = 17;
const int FRPort = 7;
const int MLPort = 18;
const int MRPort = 2;
const int BLPort = 9;
const int BRPort = 10;
const int FBRPort = 8;
const int CPort = 4;
const int IMUPort = 21;
const int ConvPort = 1;
const int FDistPort = 4;
const int BDistPort = 5;

pros::Controller control(pros::E_CONTROLLER_MASTER);

pros::Motor FrontLeft(FLPort, MOTOR_GEARSET_06, true);
pros::Motor FrontRight(FRPort, MOTOR_GEARSET_06, false);
pros::Motor BackLeft(BLPort, MOTOR_GEARSET_06, true);
pros::Motor BackRight(BRPort, MOTOR_GEARSET_06, false);


pros::Imu inertial(IMUPort);

pros::ADIEncoder Left_Enc('G', 'F');
pros::ADIEncoder Right_Enc('E', 'F');



driveTrain drive = driveTrain(3.25, 11.5, std::vector<pros::Motor> {FrontLeft,BackLeft},std::vector<pros::Motor> {FrontRight,BackRight});
pidController autonlinear = pidController(0, 0.002, 0, 0.0001);
pidController autonrotation = pidController(0, 0.001, 0, 0.0001);
pidController skillslinear = pidController(0, 0.45, 0.0, 5);
pidController skillsrotation = pidController(0, 0.30, 0.0, 5);
odomController m_odom = odomController();



void calibrateSensors() {
  lcd::print(1, "Calibrating");
  inertial.reset();

  int timeInit = millis();

  inertial.reset();
  while (inertial.is_calibrating()) {
    lcd::print(1, "Calibrating");
    delay(10);
  }
  delay(2000);
  lcd::print(1, "Calibration took %f", millis() - timeInit);

  autonlinear.tolerance = 0.2;
}
