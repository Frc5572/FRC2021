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

void Turret::TurretMove() {
    if (this->Operator->L().second > .2 || this->Operator->L().second < -.2) {
        TurretMotor->Set(-1 * Operator->L().second * .5);
    } else {
        TurretMotor->Set(0);
    }

    if (this->Operator->R().second > .2 ||  this->Operator->R().second < -.2) {
        TurretMotor->Set(-1 * Operator->R().second  * .5);
    } else {
        TurretMotor->Set(0);
    }
}

void Turret::Aim() {
    disX = LimeLight->disX;

    if (fabs(disX) > 3) {
        auto aDisX = fabs(disX);
        if (aDisX < 10)
        {
            T = .05 * copysign(1, disX);
        }
        else
        {
            T = disX/85;
        }
    } else {
        T = 0;
    }

    std::cout << "Motor set at: " << T << "\ndisX is: " << disX << "\n" ;

    TurretMotor->Set(T);
}
