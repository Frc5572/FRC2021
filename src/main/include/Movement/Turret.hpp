#include <iostream>
#include "Vision/VisionManager.hpp"
#include "Movement/ControllerManager.hpp"
#include "rev/CANSparkMax.h"
#include "AHRS.h"
#include "frc/WPILib.h"
// #include "DriveTrainManager.hpp"




class Turret {
 public:

Turret(
  rev::CANSparkMax &TopLeftMotor,
  FRC5572Controller &Operator,
  VisionManager &VisionManager
  );

~Turret();

void turretInit();
void TurretMove();
void Aim();
void autoAim();
void Off();

  VisionManager* LimeLight;

  double disX, T;

  FRC5572Controller* Operator;

  rev::CANSparkMax* TurretMotor;

};
