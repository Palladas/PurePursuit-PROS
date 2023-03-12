/*
    
    main.cpp
    Purpose: Contains all the functions used by the VRC Competition System

*/

#include "main.h"
#include "../include/autonomous.h"
#include <cmath>


/*
    Called by the VRC Competition System directly after program start
*/

void initialize() {
  pros::lcd::initialize();
  inertial.tare(); //Zeros the Inertial Measurement Unit (IMU)
  autonSelector(); //Runs a custom autonomous selector
}


/*
    Called by the VRC Competition System during the disabled period (not autonomous or operator control)
*/

void disabled() { control.clear(); }


/*
    Called by the VRC Competition System directly after program start while plugged into a VRC Competition Switch
*/

void competition_initialize() {}


/*
    Called by the VRC Competition System during the autonomous period in a match
*/

void autonomous() {

  //Uses the output from autonSelector() to run the appropriate autonomous
  switch (selected) {
  case 0:
    break;
  case 1:
    break;
  case 2:
    break;
  case 3:
    break;
  case 4:
    break;
  case 5:
    break;
  case 6:
    break;
  case 7:
    break;
  case 8:
    break;
  case 9:
    break;
  case 10:
    break;
  }
  return;
}


/*
    Function called periodically in the seperate thread (known as tasks in the PROS environment) odom_task
*/

void odomContainer(){
  m_odom.update(inertial.get_heading()*M_PI/180); //Updates the pose calulation function with the current IMU heading (converted to radians)
  pros::screen::print(TEXT_MEDIUM, 3, "Current Pos: X: %d Y: %d H: %d", m_odom.getPos().m_p.m_x, m_odom.getPos().m_p.m_y, m_odom.getPos().m_r.m_v*180/M_PI); //Outputs the current pose to the VEX brain
  
}


/*
    Called by the VRC Competition System during the operator control period in a match
*/

void opcontrol() {

  control.clear();
  Task odom_task(odomContainer); //Creates a seperate thread running the function odomContainer

  while (true) {
    printOnScreen(); //Used for debugging
    

    double power = control.get_analog(ANALOG_LEFT_Y);
    double turn = control.get_analog(ANALOG_LEFT_X);
    driverControl((power + turn), (power - turn)); //Basic arcade control for testing purposes 
    
    pros::delay(20);
  }
}
