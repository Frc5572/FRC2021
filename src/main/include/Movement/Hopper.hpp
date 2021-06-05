#ifndef HOPPER_HPP
#define HOPPER_HPP

#include "rev/CANSparkMax.h"
#include "Movement/ControllerManager.hpp"
#include <frc/SpeedControllerGroup.h>
#include <frc/DoubleSolenoid.h>
#include <frc/DigitalInput.h>
#include "ctre/Phoenix.h"

// AKA magazine and the intake

class Hopper{
    public:

    Hopper(
        WPI_TalonSRX &Belt1,
        WPI_TalonSRX &Belt2,
        FRC5572Controller &Operator
        // frc::DigitalInput &Input2,
        // frc::DigitalInput &Input3


    );

    void Run();

    // void HopperPeriodic();
    // void Advance();
    // void ManualControlBelt();
    // void RunIntakePistions();
    // void ManualIntakeMotors();

    frc::SpeedControllerGroup* HopperMotors;

    FRC5572Controller* Operator;

    WPI_TalonSRX* belt1;
    WPI_TalonSRX* belt2;
    // frc::DigitalInput* limitSwitch2;
    // frc::DigitalInput* limitSwitch3;

};

#endif
