#include "Movement/Shooter.hpp"


Shooter::Shooter(
    WPI_TalonSRX &Shooter1,
    WPI_TalonSRX &Shooter2,
    frc::DoubleSolenoid &hopperSol,
    FRC5572Controller &Operator
    ) {
    this->shooterMotors = new frc::SpeedControllerGroup(Shooter1, Shooter2);
    this->hopperBlock = &hopperSol;
    this->s1 = &Shooter1;
    this->s2 = &Shooter2;
    this->Operator = &Operator;
}


void Shooter::Run() {
    if(Operator->X()){
        shooterMotors->Set(.6);
        hopperBlock->Set(frc::DoubleSolenoid::Value::kForward);
    } else {
        shooterMotors->Set(0);
    }
}
