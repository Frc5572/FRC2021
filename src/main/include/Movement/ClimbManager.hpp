#include "rev/CANSparkMax.h"
#include "Movement/ControllerManager.hpp"
#include <frc/SpeedControllerGroup.h>
#include <frc/DoubleSolenoid.h>
#include <frc/DigitalInput.h>

// AKA magazine and the intake

class Climber{
    public:

    Climber(
        rev::CANSparkMax &m_winch1,
        rev::CANSparkMax &m_winch2,
        frc::DoubleSolenoid &climber1,
        frc::DoubleSolenoid &climber2,
        FRC5572Controller &Driver
        // frc::DigitalInput &Input2,
        // frc::DigitalInput &Input3


    );

    void RunMotors();
    void RunPistons();

    // void HopperPeriodic();
    // void Advance();
    // void ManualControlBelt();
    // void RunIntakePistions();
    // void ManualIntakeMotors();

    frc::SpeedControllerGroup* WinchMotors;

    FRC5572Controller* Driver;

    frc::DoubleSolenoid* rel1;

    frc::DoubleSolenoid* rel2;

    rev::CANSparkMax* winch1;
    rev::CANSparkMax* winch2;
    // frc::DigitalInput* limitSwitch2;
    // frc::DigitalInput* limitSwitch3;

};
