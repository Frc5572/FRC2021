#include "Movement/Hopper.hpp"

// AKA magazine and the intake
//d pad toggle preset shots

Hopper::Hopper(
    WPI_TalonSRX &Belt1,
    WPI_TalonSRX &Belt2,
    FRC5572Controller &Operator
    //    frc::DigitalInput &Input2,
    //    frc::DigitalInput &Input3
    ) {
        // this->belt1 = &Belt1;
        // this->belt2 = &Belt2;
        // this->Operator = &Operator;
        // this->limitSwitch2 = &Input2;
        // this->limitSwitch3 = &Input3;
        this->HopperMotors = new frc::SpeedControllerGroup(Belt1, Belt2);

        this->belt1 = &Belt1;
        this->belt2 = &Belt2;
        this->Operator = &Operator;
}


void Hopper::Run() {
    if(Operator->Y()) {
        HopperMotors->Set(.3);
    }
    else {
        HopperMotors->Set(0);
    }
}
