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

void Turret::turretInit() {
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
    // LimeLight->Update();
    if (Operator->X() ==  true) {
        // disX = 0;
        // T = 0;
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("camMode", 0);
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("ledMode", 3);
    } else if (Operator->Y() == false) {
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("camMode", 1);
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("ledMode", 1);
        disX = 0;
        T = 0;
    }

    disX = LimeLight->disX;
    TurretMotor->Set(disX/100);
}

void Turret::autoAim() {
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("camMode", 0);
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("ledMode", 3);
    disX = LimeLight->disX;
    TurretMotor->Set(disX/100);
}

void Turret::Off() {
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("camMode", 1);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("ledMode", 1);
    disX = 0;
    T = 0;
}

double Turret::CalculateDistance(double area) {
    auto r = m * (area) + b;
    return r;
}

double Turret::CalculateAngle(double distance) {
    auto t = tan(heightdiff / distance);
    auto d = t * (180 / M_PI);
    std::cout << "angle " << d << "\n";
    auto r = m1 * (d) + b1;
    return r;
}

void Turret::Shoot() {
    auto sShort = nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->GetNumber("tshort", 1);
    auto sLong = nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->GetNumber("tlong", 1);
    auto distance = CalculateDistance(sLong * sShort);
    auto servoPosition = CalculateAngle(distance);
    servo->Set(servoPosition);
    std::cout << distance << " dis \n";
    std::cout << servoPosition << " servo \n";
}
