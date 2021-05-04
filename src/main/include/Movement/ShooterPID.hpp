#pragma once

#include <frc/PIDController.h>
#include <frc/SpeedControllerGroup.h>
#include "ctre/Phoenix.h"
#include <frc/controller/PIDController.h>


using namespace rev;

class PIDShooter 
{
    frc2::PIDController *pid;
    frc::SpeedControllerGroup *motors;
    WPI_TalonSRX *master; 
    WPI_TalonSRX *follower;
    int rpm;

    public:
    PIDShooter(WPI_TalonSRX &m1, WPI_TalonSRX &m2, int RPM = 2000)
    {

        frc::SmartDashboard::PutNumber("P Gain", 0);
        frc::SmartDashboard::PutNumber("I Gain", 0);
        frc::SmartDashboard::PutNumber("D Gain", 0);
        frc::SmartDashboard::PutNumber("I Zone", 0);
        frc::SmartDashboard::PutNumber("Feed Forward", 0);
        frc::SmartDashboard::PutNumber("Max Output", 0);
        frc::SmartDashboard::PutNumber("Min Output", 0);
        frc::SmartDashboard::PutNumber("SetPoint", 0);
        frc::SmartDashboard::PutNumber("Auto Selection", 0);
        m1.SetInverted(true);
        master = &m1;
        follower = &m2;
        rpm = RPM;
        motors = new frc::SpeedControllerGroup(m1, m2);
        pid = new frc2::PIDController(.0045, .0, 0);
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
        // auto s1 = master->GetEncoder().GetVelocity();
        auto s2 = follower->GetSelectedSensorVelocity();
        frc::SmartDashboard::PutNumber("RPM", s2);
        // motors->Set(pid->Calczpulate(speed, rpm));
        // master->Set(pid->Calculate(s1, rpm));
        follower->Set(pid->Calculate(s2, rpm));
    }
};