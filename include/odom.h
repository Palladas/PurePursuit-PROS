/*
    
    odom.h
    Purpose: Contains the odomController class

*/



#include "main.h"
#include "point.h"
#include "driveTrain.h"

/**
 * Handles the various deltas relating to the pose exponetial computation. 
*/

class odomController{
  public:
    point currentPos;
    driveTrain drive;


    angle prevAngle;
    double prevLeftDistance;
    double prevRightDistance;

    odomController(){
      prevAngle = angle();
      prevLeftDistance = 0;
      prevRightDistance = 0;
      currentPos = point();
    }

    odomController(point initialPos, double leftVal, double rightVal){
      prevAngle = angle();
      prevLeftDistance = leftVal;
      prevRightDistance = rightVal;
      currentPos = initialPos;
    }

    void resetPosition(){
      prevAngle = angle();
      prevLeftDistance = 0;
      prevRightDistance = 0;
      currentPos = point();
    }

    point getPos(){
      return currentPos;
    }

    /**
	    * Updates the pose based on the current left/right distance traveled and current heading
	    * \param gyroAngle 
      * Current heading in radians
    */

    void update(angle gyroAngle){
      std::pair<int,int> currDistance = drive.getDistance();
      double deltaLeft = currDistance.first - prevLeftDistance;
      double deltaRight = currDistance.second - prevRightDistance;

      prevLeftDistance = currDistance.first;
      prevRightDistance = currDistance.second;

      double avgDeltaDistance = (deltaRight+deltaLeft)/2.0;

      point newPose = currentPos.exp(delta(avgDeltaDistance, 0.0, gyroAngle.minus(prevAngle).m_v));
      prevAngle = gyroAngle;

      currentPos = point(newPose.m_p, gyroAngle);
    }
};