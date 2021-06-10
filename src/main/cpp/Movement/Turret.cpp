#include "Movement/Turret.hpp"
#include "Movement/DriveTrainManager.hpp"

Turret::Turret(
    rev::CANSparkMax &TurretMotor,
    FRC5572Controller &Operator,
    VisionManager &VisionManager,
    frc::Servo &servo,
    DriveTrain &driveTrain
    ) {

        this->TurretMotor = &TurretMotor;
        this->servo = &servo;
        this->LimeLight = &VisionManager;
        this->Operator = &Operator;
        drive = &driveTrain;
        TurretEncoder = new rev::CANEncoder(TurretMotor, rev::CANEncoder::EncoderType::kHallSensor);
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
    if (Operator->RB() && abs(TurretEncoder->GetPosition()) < limitTurret) {
        TurretMotor->Set(.1);
    }
    else if (Operator->LB() && abs(TurretEncoder->GetPosition()) < limitTurret) {
        TurretMotor->Set(-.1);
    } else {
        autoAim();
        // TurretMotor->Set(0);
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
    if (abs(TurretEncoder->GetPosition()) > limitTurret) 
    {
        TurretMotor->Set(0);
    } else {
        TurretMotor->Set(disX/125);
    }

}

void Turret::autoAim() {
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("camMode", 0);
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("ledMode", 3);
    disX = LimeLight->disX;

    auto l = drive->LeftMotors->Get();
    auto r = drive->RightMotors->Get();

    auto s = (disX/125) + (l/90) - (r/90);

    if (abs(TurretEncoder->GetPosition()) > limitTurret) 
    {
        TurretMotor->Set(0);
    } else {
        TurretMotor->Set(s);
    }
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
    auto r = m1 * corrected_d  + b1;
    if (corrected_d > 64) 
    {
        r = limitServo;
    } 
    else if (corrected_d < 26)
    {
        r = 0;
    }

    return r;
}

// void Turret::LimitCheck() {
//     if (abs(TurretEncoder->GetPosition()) > limitTurret) 
//     {
//         TurretMotor->Set(0);
//     }
// }

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
    // std::cout << "Total area: " << area << "\n";
    auto a1 = atan2(heightdiff, CalculateDistance(area)) * (180/M_PI);
    // std::cout << "a1 " << a1 << "\n";
    auto a2 = 90 - a1 - 35;
    // std::cout << "a2 " << a2 << "\n";
    auto p = (1 / (maxAngle - minAngle))*(a2-maxAngle) + 1;
    // std::cout << "servo position" << p << "\n";
    if (p >= .7) {
        p = .7;
    }
    servo->Set(p);
}
