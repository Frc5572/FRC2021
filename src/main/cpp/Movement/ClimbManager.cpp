#include "Movement/ClimbManager.hpp"

// AKA magazine and the intake
//d pad toggle preset shots

Climber::Climber(
    rev::CANSparkMax &m_winch1,
    rev::CANSparkMax &m_winch2,
    frc::DoubleSolenoid &climber1,
    frc::DoubleSolenoid &climber2,
    FRC5572Controller &Driver

    //    frc::DigitalInput &Input2,
    //    frc::DigitalInput &Input3
    ) {
        // this->belt1 = &Belt1;
        // this->belt2 = &Belt2;
        // this->Operator = &Operator;
        // this->limitSwitch2 = &Input2;
        // this->limitSwitch3 = &Input3;
        this->WinchMotors = new frc::SpeedControllerGroup(m_winch1, m_winch2);
        this->rel1 = &climber1;
        this->rel2 = &climber2;
        this->winch1 = &m_winch1;
        this->winch2 = &m_winch2;
        this->Driver = &Driver;
}

void Climber::RunMotors() {
    if(Driver->A()){
        WinchMotors->Set(-.3);
    }
    else
    {
        WinchMotors->Set(0);
    }
}

void Climber::RunPistons() {
    if(Driver->Y()){
        rel1->Set(frc::DoubleSolenoid::Value::kForward);
        rel2->Set(frc::DoubleSolenoid::Value::kForward);
    }
}
