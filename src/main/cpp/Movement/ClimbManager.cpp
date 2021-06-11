#include "Movement/ClimbManager.hpp"

// AKA magazine and the intake
//d pad toggle preset shots

Climber::Climber(
    rev::CANSparkMax &m_wench1,
    rev::CANSparkMax &m_wench2,
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
        this->WenchMotors = new frc::SpeedControllerGroup(m_wench1, m_wench2);
        this->rel1 = &climber1;
        this->rel2 = &climber2;
        this->wench1 = &m_wench1;
        this->wench2 = &m_wench2;
        this->Driver = &Driver;
}

void Climber::RunMotors() {
    if(Driver->A()){
        WenchMotors->Set(-.3);
    }
    else
    {
        WenchMotors->Set(0);
    }
}

void Climber::RunPistons() {
    if(Driver->Y()){
        rel1->Set(frc::DoubleSolenoid::Value::kForward);
        rel2->Set(frc::DoubleSolenoid::Value::kForward);
    }
}
