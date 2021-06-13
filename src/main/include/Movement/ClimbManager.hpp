#include "rev/CANSparkMax.h"
#include "Movement/ControllerManager.hpp"
#include <frc/SpeedControllerGroup.h>
#include <frc/DoubleSolenoid.h>
#include <frc/DigitalInput.h>

// AKA magazine and the intake

class Climber{
    public:

    Climber(
        rev::CANSparkMax &m_brittney,
        rev::CANSparkMax &m_stephanie,
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

    frc::SpeedControllerGroup* WenchMotors;

    FRC5572Controller* Driver;

    frc::DoubleSolenoid* rel1;

    frc::DoubleSolenoid* rel2;

    rev::CANSparkMax* wench1;
    rev::CANSparkMax* wench2;
    // frc::DigitalInput* limitSwitch2;
    // frc::DigitalInput* limitSwitch3;

};
