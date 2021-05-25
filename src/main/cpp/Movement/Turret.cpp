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
    // if (Operator->X() ==  true) {
        disX = 0;
        T = 0;
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("camMode", 0);
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("ledMode", 3);
    // } else if (Operator->Y() == false) {
    //     nt::NetworkTableInstance::GetDefault().GetTable("limelight")
    //                 ->PutNumber("camMode", 1);
    //     nt::NetworkTableInstance::GetDefault().GetTable("limelight")
    //                 ->PutNumber("ledMode", 1);
    //     disX = 0;
    //     T = 0;
    // }

    disX = LimeLight->disX;

    auto l =Operator->L().second, r = Operator->R().second;

    if (l < .1)
    {
        l = 0;
    }
    if (r < .1)
    {
        r = 0;
    }
    double dir = (r - l) * .2;
    std::cout << "l is: " << l << " r is: " << r << "dir is: " << dir << "\n";

    if (fabs(disX) > 3 || fabs(dir) > .1) {
        // if (disX > 10) {
        //     T = -.10;
        // }
        // if (disX < 10) {
        //     T = -disX/60;
        // }
        // if (disX < -10) {
        //     T = .10;
        // }
        // if (disX > -10) {
        //     T = disX/60;
        // }
        auto aDisX = fabs(disX);



    if (aDisX < 10)
        {
            T = .05 * copysign(1, disX) + dir;
        }
    else
        {
            T = disX/85 + dir;
        }
    } else {
        T = 0;
    }

    std::cout << "Motor set at: " << T << "\ndisX is: " << disX << "\n" ;

    TurretMotor->Set(T);
}

void Turret::Off() {
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("camMode", 1);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("ledMode", 1);
    disX = 0;
    T = 0;
}
