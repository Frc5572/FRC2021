#pragma once

#include <frc/PIDController.h>
#include <frc/SpeedControllerGroup.h>
#include "ctre/Phoenix.h"
#include "Movement/ControllerManager.hpp"
#include <frc/controller/PIDController.h>


using namespace rev;

class PIDShooter
{
    frc2::PIDController *pid;
    WPI_TalonSRX *master;
    WPI_TalonSRX *follower;
    frc::SpeedControllerGroup *group;
    FRC5572Controller *Operator;
    frc::DoubleSolenoid *sol;
    double v;

    public:
    PIDShooter(WPI_TalonSRX &m1, WPI_TalonSRX &m2, FRC5572Controller &oper, frc::DoubleSolenoid &hopperSol, double voltage = 1)
    {

        frc::SmartDashboard::PutNumber("P Gain", 0);
        frc::SmartDashboard::PutNumber("I Gain", 0);
        frc::SmartDashboard::PutNumber("D Gain", 0);
        frc::SmartDashboard::PutNumber("SetPoint", voltage);
        m1.SetInverted(true);
        master = &m1;
        follower = &m2;
        v = voltage;
        pid = new frc2::PIDController(0, 0, 0);
        group = new frc::SpeedControllerGroup(m1, m2);
        Operator = &oper; 
        sol = &hopperSol;
    }

    void run()
    {
        //0.000060 for p
        //0.000001 for i
        // motors->Get()
        frc::SmartDashboard::PutNumber("Voltage", group->Get());
        if(Operator->RT()){
            double p = frc::SmartDashboard::GetNumber("P Gain", 0);
            double i = frc::SmartDashboard::GetNumber("I Gain", 0);
            double d = frc::SmartDashboard::GetNumber("D Gain", 0);
            double v = frc::SmartDashboard::GetNumber("SetPoint", 0);
            pid->SetPID(p, i, d);
            double s = pid->Calculate(group->Get(), v);
            group->Set(s);
            sol->Set(frc::DoubleSolenoid::Value::kForward);
        }
        else
        {
            group->Set(0);
            sol->Set(frc::DoubleSolenoid::Value::kReverse);
        }
        // follower->Follow(*master);
    }
};
