#pragma once

#include <frc/PIDController.h>
#include <frc/SpeedControllerGroup.h>
#include "ctre/Phoenix.h"
#include <frc/controller/PIDController.h>


using namespace rev;

class PIDShooter
{
    frc2::PIDController *pid;
    WPI_TalonSRX *master;
    WPI_TalonSRX *follower;
    int rpm;

    public:
    PIDShooter(WPI_TalonSRX &m1, WPI_TalonSRX &m2, int RPM = 2000)
    {

        frc::SmartDashboard::PutNumber("P Gain", 0);
        frc::SmartDashboard::PutNumber("I Gain", 0);
        frc::SmartDashboard::PutNumber("D Gain", 0);
        frc::SmartDashboard::PutNumber("SetPoint", 0);
        m1.SetInverted(true);
        master = &m1;
        follower = &m2;
        rpm = RPM;
        pid = new frc2::PIDController(.0045, .0, 0, 100_ms);
        // frc::SmartDashboard::PutBoolean("Shooter Motor 1 or 2", true);
    }

    void run()
    {
        //0.000060 for p
        //0.000001 for i
        // motors->Get()
        double p = frc::SmartDashboard::GetNumber("P Gain", 0);
        double i = frc::SmartDashboard::GetNumber("I Gain", 0);
        double d = frc::SmartDashboard::GetNumber("D Gain", 0);
        rpm = frc::SmartDashboard::GetNumber("SetPoint", 0);
        pid->SetPID(p, i, d);
        // auto speed = (master->GetEncoder().GetVelocity() + follower->GetEncoder().GetVelocity()) / 2;
        auto s1 = master->GetSelectedSensorVelocity();
        auto s2 = follower->GetSelectedSensorVelocity();
        auto speed = (s1 + s2) / 2;
        frc::SmartDashboard::PutNumber("RPM", speed);
        // motors->Set(pid->Calczpulate(speed, rpm));
        std::cout << "Speed 1: " << s1 << "\n";
        std::cout << "Speed 2: " << s2 << "\n";

        auto v1 = pid->Calculate(s1, rpm);
        auto v2 = pid->Calculate(s2, rpm);

        auto ceiling = .5;
        if (v1 > ceiling)
            v1 = ceiling;
        if (v2 > ceiling)
            v2 = ceiling;
        if (v1 < -ceiling)
            v1 = -ceiling;
        if (v2 < -ceiling)
            v2 = -ceiling;

        std::cout << "Voltage 1: " << v1 << "\n";
        std::cout << "Voltage 2: " << v2 << "\n";
        
        master->Set(v1);
        follower->Set(v2);
        // follower->Follow(*master);
    }
};
