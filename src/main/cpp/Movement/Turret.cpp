#include "Movement/Turret.hpp"
#include "Movement/DriveTrainManager.hpp"

bool tap = false;

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
    if (Operator->RB()) {
        TurretMotor->Set(.1);
    }
    else if (Operator->LB()) {
        TurretMotor->Set(-.1);
    } else {
        autoAim();
        // TurretMotor->Set(0);
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

    auto s = (disX/125) - (l/90) + (r/90);

    TurretMotor->Set(s);
}

void Turret::Off() {
    // if (Operator.X() && tap == false){}
    //     nt::NetworkTableInstance::GetDefault().GetTable("limelight")
    //                     ->PutNumber("camMode", 1);
    //     nt::NetworkTableInstance::GetDefault().GetTable("limelight")
    //                     ->PutNumber("ledMode", 1);
    //     disX = 0;
    //     T = 0;
    //     tap = true;
    // } else if (Operator.X() && tap == true) {
    //     nt::NetworkTableInstance::GetDefault().GetTable("limelight")
    //                 ->PutNumber("camMode", 0);
    //     nt::NetworkTableInstance::GetDefault().GetTable("limelight")
    //                 ->PutNumber("ledMode", 3);
    //     tap = false;
    // }
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

bool Turret::LimitCheck() {
    if (abs(TurretEncoder->GetPosition()) > limitTurret)
    {
        return true;
    }
    return false;
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
    // std::cout << "Total area: " << area << "\n";
    std::cout << CalculateDistance(area) << "inches\n";
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
