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
        frc::DoubleSolenoid &intakeSol,
        WPI_TalonSRX &m_hopperLeft,
        WPI_TalonSRX &m_hopperRight,
        frc::DoubleSolenoid &hopperSol,
        FRC5572Controller &Operator
        // frc::DigitalInput &Input2,
        // frc::DigitalInput &Input3


    );

    void Run();
    // void RunIntakePistions();
    // void ManualIntakeMotors();


    FRC5572Controller* Operator;

    frc::DoubleSolenoid* intakePistons;

    frc::SpeedControllerGroup* HopperMotors;

    WPI_TalonSRX* m_intake;
    WPI_TalonSRX* h1;
    WPI_TalonSRX* h2;


    frc::DoubleSolenoid *hS;


    // frc::DigitalInput* limitSwitch2;
    // frc::DigitalInput* limitSwitch3;

};
