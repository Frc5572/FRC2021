#ifndef DRIVE_TRAIN_HPP
#define DRIVE_TRAIN_HPP

#include <iostream>
#include "Vision/VisionManager.hpp"
#include "Movement/ControllerManager.hpp"
#include <frc/SpeedControllerGroup.h>
#include "rev/CANSparkMax.h"
#include "AHRS.h"
#include "ctre/Phoenix.h"
#include "frc/WPILib.h"


/* CAN ID layout for drive train from a top view

          Front of Robot

         |--------------|
         |              |
      1  | M1        M2 | 2
      3  | M3        M4 | 3
      5  | M5        M6 | 6
         |              |
         |              |
         |--------------|

          Back Of Robot
*/

class DriveTrain {
 public:
DriveTrain(
  WPI_TalonSRX &TopLeftMotor     ,
  WPI_TalonSRX &TopRightMotor    ,
  WPI_TalonSRX &MiddleLeft       ,
  WPI_TalonSRX &MiddleRight      ,
  WPI_TalonSRX &BottomLeftMotor  ,
  WPI_TalonSRX &BottomRightMotor ,
  FRC5572Controller &Driver          ,
  VisionManager &VisionManager      ,
  AHRS &ahrs);

~DriveTrain();
void Drive();
void LowerAmps();
void Aim();

  VisionManager* LimeLight;

  double disX, L, R;

  frc::SpeedControllerGroup* LeftMotors;
  frc::SpeedControllerGroup* RightMotors;
  frc::SpeedControllerGroup* TempRightMotors;
  frc::SpeedControllerGroup* TempLeftMotors;

  FRC5572Controller* Driver;

  WPI_TalonSRX* TopLeftMotor;
  WPI_TalonSRX* TopRightMotor;

  WPI_TalonSRX* MiddleLeft;
  WPI_TalonSRX* MiddleRight;

  WPI_TalonSRX* BottomLeftMotor;
  WPI_TalonSRX* BottomRightMotor;


  AHRS* ahrs;
};
#endif
