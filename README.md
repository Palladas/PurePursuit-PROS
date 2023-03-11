# PurePursuit-PROS

Implementation of pure pursuit for use in VRC (Vex Robotics Competitions). An adaptive pure pursuit controller allows a robot to follow a path accurately with the ability to correct for variances in fields and poorly tuned parameters. Unlike motion profiling, pure pursuit returns target velocities based on where the robot is in relation to the path it wants to follow.

Currently, this repository contains an implementation of odometry via the pose exponential for use in pure pursuit. This implementation of the pose exponential uses first-order dynamics to approximate the pose from only the change in left and right encoders and change in heading. The widely used implementation of odometry in VRC requires additional sensors and seems to be less accurate than this method.  

[src/main.cpp](https://github.com/Palladas18/PurePursuit-PROS/blob/main/src/main.cpp): Contains the function calls used by the VRC competition system

[include/](https://github.com/Palladas18/PurePursuit-PROS/tree/main/include): Contains all dependencies and .h files used by the code
