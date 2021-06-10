#include "Movement/Shooter.hpp"


Shooter::Shooter(
    WPI_TalonSRX &Shooter1,
    WPI_TalonSRX &Shooter2,
    frc::DoubleSolenoid &hopperSol,
    FRC5572Controller &Operator
    ) {
    this->shooterMotors = new frc::SpeedControllerGroup(Shooter1, Shooter2);
    this->hopperBlock = &hopperSol;
    this->shoot1 = &Shooter1;
    this->shoot2 = &Shooter2;
    this->Operator = &Operator;
}
