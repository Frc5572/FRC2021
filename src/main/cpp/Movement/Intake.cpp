#include "Movement/Intake.hpp"

// AKA magazine and the intake
//d pad toggle preset shots

Intake::Intake(
    WPI_TalonSRX &m_intake,
    FRC5572Controller &Operator
    //    frc::DigitalInput &Input2,
    //    frc::DigitalInput &Input3
    ) {
        // this->belt1 = &Belt1;
        // this->belt2 = &Belt2;
        // this->Operator = &Operator;
        // this->limitSwitch2 = &Input2;
        // this->limitSwitch3 = &Input3;
        this->m_intake = &m_intake;
        this->Operator = &Operator;
}

Intake::~Intake() {
}

// void Intake::Run() {
//     if(Operator->A()){
//         m_intake->Set(.7);
//     }
//     else {
//         m_intake->Set(0);
//     }
// }
