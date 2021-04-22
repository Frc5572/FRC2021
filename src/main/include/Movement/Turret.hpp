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
#include "DriveTrainManager.hpp"




class Turret {
 public:

Turret(
  rev::CANSparkMax &TopLeftMotor,
  FRC5572Controller &Driver,
  VisionManager &VisionManager
  );

~Turret();


  VisionManager* LimeLight;

  double disX, T;

  FRC5572Controller* Driver;

  rev::CANSparkMax* TurretMotor;

};
#endif
