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

    ~Shooter();

    void Run();


    frc::SpeedControllerGroup* shooterMotors;

    FRC5572Controller* Operator;

    frc::DoubleSolenoid* hopperBlock;

    WPI_TalonSRX* s1;
    WPI_TalonSRX* s2;
};
