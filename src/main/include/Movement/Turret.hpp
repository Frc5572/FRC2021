#include <iostream>
#include "Vision/VisionManager.hpp"
#include "Movement/ControllerManager.hpp"
#include "rev/CANSparkMax.h"
#include "AHRS.h"
#include "frc/WPILib.h"
#include "Movement/DriveTrainManager.hpp"
#include "rev/CANEncoder.h"




class Turret {
// Distance calculation constants
    static constexpr double x1 = -0.0000000005671;
    static constexpr double x2 = 0.0000119179047;
    static constexpr double x3 = -0.0903309053502;
    static constexpr double b = 317.9789050449609;
    static constexpr double heightOfShooter = 38;
    static constexpr double heightOfTower = 98;
    static constexpr double heightdiff = heightOfTower - heightOfShooter;
    static constexpr double minAngle = 25;
    static constexpr double maxAngle = 65;
    static constexpr double maxPosition = 0;
    static constexpr double minPosition = 1;
    static constexpr double m1 = -(maxPosition - minPosition) / (maxAngle - minAngle);
    static constexpr double b1 = -.625;
    static constexpr double limitTurret = 20;
    static constexpr double limitServo = .7;

 public:

Turret(
  rev::CANSparkMax &TopLeftMotor,
  FRC5572Controller &Operator,
  VisionManager &VisionManager,
  frc::Servo &servo,
  DriveTrain &driveTrain
  );

void turretInit();
void TurretMove();
// void Aim();
void autoAim();
void Off();
// void Shoot();
void PositionHood();
bool LimitCheck();
double CalculateDistance(double area);
double CalculateAngle(double position);

  VisionManager* LimeLight;

  double disX, T;

  FRC5572Controller* Operator;

  rev::CANSparkMax* TurretMotor;

  rev::CANEncoder *TurretEncoder;

  frc::Servo *servo;

  DriveTrain *drive;

};
