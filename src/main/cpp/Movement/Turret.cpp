#include "Robot.h"
#include "Movement/Turret.hpp"

Turret::Turret(
    rev::CANSparkMax &TurretMotor,
    FRC5572Controller &Operator,
    VisionManager &VisionManager
    ) {

        this->TurretMotor = &TurretMotor;

        this->LimeLight = &VisionManager;
        this->Operator = &Operator;
}

Turret::~Turret() {
    delete TurretMotor;
}

void Turret::Turret() {
    if (this->Driver->L().second > .2 || this->Driver->L().second < -.2) {
        LeftMotors->Set(-1 * Driver->L().second * .5);
    } else {
        LeftMotors->Set(0);
    }

    if (this->Driver->R().second > .2 ||  this->Driver->R().second < -.2) {
        RightMotors->Set(-1 * Driver->R().second  * .5);
    } else {
        RightMotors->Set(0);
    }
}

void Turret::Aim() {
    if (Driver->X() ==  true) {
        disX = LimeLight->disX;
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("camMode", 0);
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("ledMode", 3);
    } else if (Driver->Y() == false) {
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("camMode", 1);
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("ledMode", 1);
        disX = 0;
        T = 0;
    }

    if (fabs(disX) > 1 && Driver->X() == true) {
        if (disX > 10) {
            T = -.15;
        }
        if (disX < 10) {
            T = -disX/43;
        }
        if (disX < -10) {
            T = .15;
        }
        if (disX > -10) {
            T = disX/43;
        }
    } else {
        T = 0;
    }
}
