#include <frc/DoubleSolenoid.h>
#include "rev/CANSparkMax.h"
#include "AHRS.h"
#include "Movement/ControllerManager.hpp"
#include <frc/SpeedControllerGroup.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"

class Shooter{
 public:
    Shooter(
        WPI_TalonSRX &Shooter1,
        WPI_TalonSRX &Shooter2,
        frc::DoubleSolenoid &hopperSol,
        FRC5572Controller &Operator
    );

    void Run();
    void retractSol();


    frc::SpeedControllerGroup* shooterMotors;

    FRC5572Controller* Operator;

    frc::DoubleSolenoid* hopperBlock;

    WPI_TalonSRX* shoot1;
    WPI_TalonSRX* shoot2;
};
