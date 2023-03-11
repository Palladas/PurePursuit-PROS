#include "main.h"
#include "../include/autonomous.h"
#include <cmath>

void leftBtn() {}
void centerBtn() {}
void rightBtn() {}
void initialize() {
  pros::lcd::initialize();
  inertial.tare();
  autonSelector();
}

void disabled() { control.clear(); }

void competition_initialize() {}

void autonomous() {
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



void odomContainer(){
  m_odom.update(inertial.get_heading()*M_PI/180);
  pros::screen::print(TEXT_MEDIUM, 3, "Current Pos: X: %d Y: %d H: %d", m_odom.getPos().m_p.m_x, m_odom.getPos().m_p.m_y, m_odom.getPos().m_r.m_v*180/M_PI);
  
}

void opcontrol() {

  control.clear();

  while (true) {
    printOnScreen();
    Task odom_task(odomContainer);

    double power = control.get_analog(ANALOG_LEFT_Y);
    double turn = control.get_analog(ANALOG_LEFT_X);
    driverControl((power + turn), (power - turn));
    
    pros::delay(20);
  }
}
