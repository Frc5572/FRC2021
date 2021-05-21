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

    static constexpr int UPR = 2048;

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

    void refresh()
    {
        double p = frc::SmartDashboard::GetNumber("P Gain", 0);
        double i = frc::SmartDashboard::GetNumber("I Gain", 0);
        double d = frc::SmartDashboard::GetNumber("D Gain", 0);
        rpm = frc::SmartDashboard::GetNumber("SetPoint", 0);
        std::cout << "Rpm is: " << rpm << "\n";
        pid->SetPID(p, i, d);
    }

    void run()
    {
        //0.000060 for p
        //0.000001 for i

        // double k = (double)rpm / 12000;
        double k = (double)rpm / 9000;


        auto s1 = master->GetSelectedSensorVelocity();
        auto s2 = follower->GetSelectedSensorVelocity();

        auto c = master->Get();
        double mc = .4-(c*.4);
        auto speed = (s1 + s2) / 2;
        // frc::SmartDashboard::PutNumber("RPM", speed);
;
        auto result = pid->Calculate(speed, rpm);

        follower->Set(result);
        master->Set(result);
    }
};