#include "Movement/Intake.hpp"

// AKA magazine and the intake
//d pad toggle preset shots

Intake::Intake(
    WPI_TalonSRX &m_intake,
    frc::DoubleSolenoid &intakeSol,
    WPI_TalonSRX &m_hopperLeft,
    WPI_TalonSRX &m_hopperRight,
    frc::DoubleSolenoid &hopperSol,
    FRC5572Controller &Operator
    //    frc::DigitalInput &Input2,
    //    frc::DigitalInput &Input3
    ) {
        // this->belt1 = &Belt1;
        // this->belt2 = &Belt2;
        // this->Operator = &Operator;
        // this->limitSwitch2 = &Input2;
        // this->limitSwitch3 = &Input3;
        this->h1 = &m_hopperLeft;
        this->h2 = &m_hopperRight;
        this->hS = &hopperSol;
        this->intakePistons = &intakeSol;
        this->m_intake = &m_intake;
        this->HopperMotors = new frc::SpeedControllerGroup(m_hopperLeft, m_hopperRight);
        this->Operator = &Operator;
}

void Intake::Run() {
    if(Operator->B()){
        m_intake->Set(.3);
        intakePistons->Set(frc::DoubleSolenoid::Value::kReverse);
        HopperMotors->Set(.4);
    }
    else {
        m_intake->Set(0);
        intakePistons->Set(frc::DoubleSolenoid::Value::kForward);
        // HopperMotors->Set(0);
    }
}

