#include "rev/CANSparkMax.h"
#include "Movement/ControllerManager.hpp"
#include <frc/SpeedControllerGroup.h>
#include <frc/DoubleSolenoid.h>
#include <frc/DigitalInput.h>
#include "ctre/Phoenix.h"

// AKA magazine and the intake

class Intake{
    public:

    Intake(
        WPI_TalonSRX &m_intake,
        FRC5572Controller &Operator,
        frc::DoubleSolenoid &sol
        // frc::DigitalInput &Input2,
        // frc::DigitalInput &Input3


    );


    void Run();

    // void HopperPeriodic();
    // void Advance();
    // void ManualControlBelt();
    // void RunIntakePistions();
    // void ManualIntakeMotors();


    FRC5572Controller* Operator;

    WPI_TalonSRX* m_intake;

    frc::DoubleSolenoid *sol;
    // frc::DigitalInput* limitSwitch2;
    // frc::DigitalInput* limitSwitch3;

};
