/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>

class MTPath {
  double _ticksPerInch;

 public:
  MTPath(double ticksPerInch){ _ticksPerInch = ticksPerInch;};
  MTPath(double wheelDiameter, double ticksPerMotorRev, double gearRatio=1.0);
  ~MTPath(){};
  
  enum column{
      position_left, 
      velocity_left, 
      position_right, 
      velocity_right, 
      duration
      };
  enum column2{
      position_left_v2, 
      velocity_left_v2, 
      position_right_v2, 
      velocity_right_v2, 
      position_center_v2, 
      velocity_center_v2, 
      heading_v2, 
      duration_v2
      };
  
  void GetStreamFromArray(BufferedTrajectoryPointStream& stream, const double traj[][5], int trajLen, bool left, bool forward);
  void GetStreamFromArray(BufferedTrajectoryPointStream& stream, const double traj[][8], int trajLen, bool left);
  
};
