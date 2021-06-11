#pragma once

#include <string>

#include "Vision/ColorManager.h"
#include "Vision/VisionManager.hpp"
#include "Movement/ControllerManager.hpp"
#include "Movement/DriveTrainManager.hpp"
#include "Movement/Shooter.hpp"
#include "Movement/Hopper.hpp"
#include "Movement/Turret.hpp"
#include "Movement/Intake.hpp"
// #include "Movement/Solenoids.hpp"
#include "Movement/ClimbManager.hpp"
#include "vision/PhotoeletricSensor.hpp"
#include <frc/SpeedControllerGroup.h>
#include <frc/Servo.h>
// #include "Auto/AutoDrip.hpp

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Compressor.h>
// #include <frc/DoubleSolenoid.h>
#include <frc/util/color.h>
#include <frc/DigitalInput.h>
#include <frc/Timer.h>
#include <frc/livewindow/LiveWindow.h>
#include <AHRS.h>
// #include <frc/Servo.h>
// #include "rev/CANSparkMax.h"
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/VictorSP.h>

#include "Movement/ShooterPID.hpp"



class Robot : public frc::TimedRobot {
 private:
    //  AutoMovement *automovement;

    // Controllers
    FRC5572Controller Driver{0};
    FRC5572Controller Operator{1};

    //  Nav-XMP boardP
    AHRS ahrs{frc::SPI::Port::kMXP};

    WPI_TalonSRX m_leftTopMotor{TopLeft};
    WPI_TalonSRX m_leftMiddleMotor{MiddleLeft};
    WPI_TalonSRX m_leftBottomMotor{BottomLeft};
    WPI_TalonSRX m_rightTopMotor{TopRight};
    WPI_TalonSRX m_rightMiddleMotor{MiddleRight};
    WPI_TalonSRX m_rightBottomMotor{BottomRight};

    WPI_TalonSRX m_intake{intakeMotor};
    WPI_TalonSRX m_hopperLeft{HopperOneID};
    WPI_TalonSRX m_hopperRight{HopperTwoID};
    WPI_TalonSRX m_shooter1{shooter1};
    WPI_TalonSRX m_shooter2{shooter2};
    // WPI_TalonSRX m_hopperLift{hopperLift};

    // WPI_TalonSRX*  m_leftTopMotor = new WPI_TalonSRX(TopLeft);
    // WPI_TalonSRX*  m_leftMiddleMotor = new WPI_TalonSRX(MiddleLeft);
    // WPI_TalonSRX*  m_leftBottomMotor = new WPI_TalonSRX(LeftBot);
    // WPI_TalonSRX*  m_rightTopMotor = new WPI_TalonSRX(TopRight);
    // WPI_TalonSRX*  m_rightMiddleMotor = new WPI_TalonSRX(MiddleRight);
    // WPI_TalonSRX*  m_rightBottomMotor = new WPI_TalonSRX(RightBot);


    // rev::CANSparkMax m_rightBottomMotor{RightBot,
        // rev::CANSparkMax::MotorType::kBrushless};

    // rev::CANEncoder* BottomLeftMotorEncoder =
        // new rev::CANEncoder{m_leftBottomMotor};


    rev::CANSparkMax m_turret{turretID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_climber1{climb1, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_climber2{climb2, rev::CANSparkMax::MotorType::kBrushless};
    frc::Servo s1{9};
    // frc::Servo s2{9};




/* Hopper */
    // rev::CANSparkMax m_turret{turretID,
    //    rev::CANSparkMax::MotorType::kBrushless};

    /*instantiation of the compressor with its CAN ID and pneumatics*/
    frc::Compressor compressor{PCM1};

    // frc::Servo *servo;
    // servo = new frc::Servo{0};

    frc::DoubleSolenoid climber2{PCM1, 7 ,0};
    frc::DoubleSolenoid *sol4;
    frc::DoubleSolenoid intakeSol{PCM1, 6, 1};
    frc::DoubleSolenoid climber1{PCM2, 4, 3};
    frc::DoubleSolenoid hopperSol {PCM1, 5, 2};

    PIDShooter shooter{m_shooter1, m_shooter2};

    Shooter nonPidShooter { m_shooter1, m_shooter2, hopperSol, Operator};

    // frc::DoubleSolenoid climb{PCM1, 1, 6};  // 3 4

    // frc::DoubleSolenoid shooterHood{PCM1, 2, 5};

    // Sensor
    // frc::DigitalInput limitSwitch2{2};  // in the mag
    // frc::DigitalInput limitSwitch3{0};  // on the top of the mag

    //  frc::DigitalInput photoIN{0};
    //  frc::DigitalOutput photoOUT{1};

    /*SubSystem Objects  */
    DriveTrain driveTrain{ m_leftTopMotor, m_rightTopMotor,
        m_leftMiddleMotor, m_rightMiddleMotor, m_leftBottomMotor,
        m_rightBottomMotor, Driver, LimeLight, ahrs };

    Hopper hopper{ m_hopperLeft, m_hopperRight, hopperSol, Operator };

    Intake intake{ m_intake, intakeSol, Operator };

    Turret turret{ m_turret, Operator, LimeLight, s1, driveTrain};

    Climber climber{ m_climber1, m_climber2, climber1, climber2, Driver };

    bool firstPart, secondPart, thirdPart, fourthPart, fifthPart;

    // bool firstPart, secondPart, thirdPart, fourthPart, fifthPart, sixthPart;

    // Shooter shooter{m_leftShooter, m_rightShooter, shooterHood, Operator};

    // ClimbManager climber{test1, test2, Driver, climb};

    //Hopper hopper{m_hopperLeft, m_hopperRight,  Operator};

    VisionManager LimeLight;

    frc::Timer m_timer;

    //  AutoDrip autoDrip{hopper, driveTrain, shooter, LimeLight};

    //  Photoelctric photoSensor{photoIN, photoOUT};

    /*  IDS  */
    static const int
    TopLeft = 4,  //  GOOD
    TopRight = 2,  //  GOOD

    MiddleLeft = 6,  //  GOOD
    MiddleRight = 3,  //  GOOD

    BottomLeft = 8,  //  GOOD
    BottomRight = 7, //  GOOD
    // HopperLift = 9,

    // LeftShoot = 7,  //  GOOD
    // RightShoot = 8,  //  GOOD

    // Intake = 9,  //  GOOD

    PCM1 = 0,  //  GOOD
    PCM2 = 1,
    turretID = 13,
    intakeMotor = 11,
    HopperOneID = 9,
    HopperTwoID = 10,
    shooter1 = 12,
    shooter2 = 14, //  GOOD
    climb1 = 16,
    climb2 = 15;

 public:
        void RobotInit() override;
        void RobotPeriodic() override;
        void AutonomousInit() override;
        void AutonomousPeriodic() override;
        void TeleopInit() override;
        void TeleopPeriodic() override;
        void TestInit() override;
        void TestPeriodic() override;
};

//  ¶¶¶¶¶¶¶¶¶¶´¶¶¶¶¶¶¶
//  ¶¶¶¶¶¶¶¶__________¶¶
//  ¶¶¶¶¶_______________¶¶¶
//  ¶¶¶____________________¶
//  ¶¶______¶¶______________¶
//  ¶______¶¶¶¶______________¶
//  ¶______¶¶¶¶____¶¶¶¶______¶
//  ¶_______¶¶_______________¶
//  ¶________________________¶
//  ¶_____¶¶__________¶¶_____¶
//  ¶¶_____¶¶________¶¶_____¶
//  ¶¶¶______¶¶¶¶¶¶¶¶¶_____¶
//  ¶¶¶¶¶________________¶¶
//  ¶¶¶¶¶¶¶¶___________¶¶
//  ¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶
