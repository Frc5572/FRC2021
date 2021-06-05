#include "Movement/Turret.hpp"

Turret::Turret(
    rev::CANSparkMax &TurretMotor,
    FRC5572Controller &Operator,
    VisionManager &VisionManager,
    frc::Servo &servo
    ) {

        this->TurretMotor = &TurretMotor;
        this->servo = &servo;
        this->LimeLight = &VisionManager;
        this->Operator = &Operator;
}

void Turret::turretInit() {
}

void Turret::TurretMove() {
    // if (this->Operator->L().second > .2 || this->Operator->L().second < -.2) {
    //     TurretMotor->Set(-1 * Operator->L().second * .5);
    // } else {
    //     TurretMotor->Set(0);
    // }

    // if (this->Operator->R().second > .2 ||  this->Operator->R().second < -.2) {
    //     TurretMotor->Set(-1 * Operator->R().second  * .5);
    // } else {
    //     TurretMotor->Set(0);
    // }
    if (Operator->RB()) {
        TurretMotor->Set(.1);
    }
    else if (Operator->LB()) {
        TurretMotor->Set(-.1);
    } else {
        autoAim();
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
    auto r = x1*pow(area, 3) + x2*pow(area, 2) +x3*area + b;
    return r;
}

double Turret::CalculateAngle(double distance) {
    auto t = atan(heightdiff / distance);
    auto d = t * (180 / M_PI);
    auto corrected_d = (90 - d - 25);
    std::cout << "angle " << corrected_d << "\n";
    auto r = m1 * corrected_d  + b1;
    std::cout << "servo pos: " << r << "\n";
    if (corrected_d > 64) 
    {
        r = 1;
    } 
    else if (corrected_d < 26)
    {
        r = 0;
    }

    return r;
}

// void Turret::Shoot() {
//     auto sShort = nt::NetworkTableInstance::GetDefault().GetTable("limelight")
//                     ->GetNumber("tshort", 1);
//     auto sLong = nt::NetworkTableInstance::GetDefault().GetTable("limelight")
//                     ->GetNumber("tlong", 1);
//     auto area = sLong * sShort;
//     std::cout << "area: " << area << "\n";
//     auto distance = CalculateDistance(area);
//     auto servoPosition = CalculateAngle(distance);
//     std::cout << "distance: " << distance << "\n";
//     std::cout << "servo Position: " << servoPosition << "\n";
//     servo->Set(servoPosition);
// }

void Turret::PositionHood()
{
    auto sShort = nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->GetNumber("tshort", 1);
    auto sLong = nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->GetNumber("tlong", 1);
    auto area = sLong * sShort;
    auto a1 = atan2(heightdiff, CalculateDistance(area)) * (180/M_PI);
    std::cout << "a1 " << a1 << "\n";
    auto a2 = 90 - a1 - 35;
    std::cout << "a2 " << a2 << "\n";
    auto p = (1 / (maxAngle - minAngle))*(a2-maxAngle) + 1;
    std::cout << "servo position" << p << "\n";
    if (p >= .7) {
        p = .7;
    }
    servo->Set(p);
}