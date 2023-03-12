/*
    
    driveTrain.h
    Purpose: Contains the driveTrain class used for quick computation and application of motor speeds (Differential Drive ONLY)

*/

#include "main.h"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include <cmath>
#include <utility>

class driveTrain{
  public:

    //Motors grouped by side
    std::vector<pros::Motor> leftSide;
    std::vector<pros::Motor> rightSide;

    //Constructors for various use cases
    driveTrain() = default;
    driveTrain(double wheelDiameter, double trackWidth, std::vector<pros::Motor> leftMotors, std::vector<pros::Motor> rightMotors):
      wheelCircumfrence(wheelDiameter * M_PI), wheelBaseCircumfrence(trackWidth * M_PI), trackWidth(trackWidth), leftSide(leftMotors),rightSide(rightMotors)
    {

    }


    //Constants for use in driveTrain
    double root2 = sqrt(2);
    double DPS2RPM = 6; // change realative motor cartridge eg 2 for blue and 12 for red
    double degToRad = M_PI / 180;
    double wheelCircumfrence;
    double trackWidth;
    double wheelBaseCircumfrence;
    double maxRPM = 200; // change realative motor cartridge eg 600 for blue and 100 for red
    double gearScale = 1; //change based on gearing
    double ticksPerRev = 900; //change based on motor type

    //Wheel speeds in RPM
    double leftRPM;
    double rightRPM;


    //xSpeed in in/s
    //rSpeed in radians/s

    /**
	    * Computes the coresponding left and right RPM from linear and rotational speeds
	    * \param xSpeed 
      * The linear speed in in/s
      * \param rSpeed 
      * The rotational speed in radians/s
    */
    void calculateWheelSpeeds(double xSpeed, double rSpeed){

  		double leftSpeed = xSpeed - trackWidth/2*rSpeed;
      double rightSpeed = xSpeed + trackWidth/2*rSpeed;

      leftRPM = ((leftSpeed*60)/(wheelCircumfrence*gearScale));
      rightRPM = ((rightRPM*60)/(wheelCircumfrence*gearScale));

      double maxsetRPM = std::max(abs(leftRPM),abs(rightRPM));
      if(maxsetRPM>maxRPM){
        leftRPM = leftRPM/maxsetRPM * maxRPM;
        rightRPM = rightRPM/maxsetRPM * maxRPM;
      }

    }

    /**
	    * Applys the current computed RPM to all motors
    */

    void applyWheelSpeeds(){
      for(auto i:leftSide){
        i.move_velocity(leftRPM);
      }
      for(auto i:rightSide){
        i.move_velocity(rightRPM);
      }
    }

    /**
	    * Returns a pair of the total distance both sides have traveled
      * \return Pair of doubles that correspond to the left distance traveled and right distance traveled
    */

    std::pair<double, double> getDistance(){
      double leftDist;
      double rightDist;
      for(auto i: leftSide){
        leftDist+=encToDist(i.get_position());
      }
      leftDist/=leftSide.size();

      for(auto i: rightSide){
        rightDist+=encToDist(i.get_position());
      }
      rightDist/=rightSide.size();
      return std::pair<double, double> {leftDist,rightDist};

    }

    /**
	    * Returns the distance corresponding to the encoder value
      * \param enc Value of the encodes (in the range of 0 - ticksPerRev)
      * \return Corresponding distance in inches
    */

    double encToDist(double enc){
      return enc*wheelCircumfrence*gearScale/ticksPerRev;
    }
  private:

    double getSign(double input){
      if(input == 0) return 0;
      return abs(input)/input;
    }
    double abs(double input){
      if(input > 0) return input;
      return -input;
    }
};
