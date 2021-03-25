#pragma once

#include <string>

#include "Vision/ColorManager.h"
#include "Vision/VisionManager.hpp"
#include "Movement/ControllerManager.hpp"
#include "Movement/DriveTrainManager.hpp"
#include "Movement/Shooter.hpp"
#include "Movement/Hopper.hpp"
#include "Movement/ClimbManager.hpp"
#include "vision/PhotoeletricSensor.hpp"
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

// #include "rev/CANSparkMax.h"
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/VictorSP.h>
#include <frc/SpeedControllerGroup.h>



class Robot : public frc::TimedRobot {
 private:
    //  AutoMovement *automovement;
    static constexpr double TANK_WHEEL_CIRCUMFERENCE = 0.1524;
    static constexpr double TANK_GEAR_RATIO = 9.0;
    static constexpr double TANK_ONE_ROTATION = TANK_GEAR_RATIO * 2048;
    static constexpr double TANK_ONE_METER = (1.0 / TANK_WHEEL_CIRCUMFERENCE) * TANK_ONE_ROTATION;
    // Controllers
    FRC5572Controller Driver{0};
    FRC5572Controller Operator{1};

    //  Nav-XMP boardP
    AHRS ahrs{frc::SPI::Port::kMXP};


    WPI_TalonSRX leftTopMotor{TopLeft};
    WPI_TalonSRX leftMiddleMotor{MiddleLeft};
    WPI_TalonSRX leftBottomMotor{BottomLeft};
    WPI_TalonSRX rightTopMotor{TopRight};
    WPI_TalonSRX rightMiddleMotor{MiddleRight};
    WPI_TalonSRX rightBottomMotor{BottomRight};
    frc::SpeedControllerGroup left{leftTopMotor, leftMiddleMotor, leftBottomMotor};
    frc::SpeedControllerGroup right{leftTopMotor, leftMiddleMotor, leftBottomMotor};


    WPI_TalonSRX m_leftTopMotor{TopLeft};
    WPI_TalonSRX m_leftMiddleMotor{MiddleLeft};
    WPI_TalonSRX m_leftBottomMotor{BottomLeft};
    WPI_TalonSRX m_rightTopMotor{TopRight};
    WPI_TalonSRX m_rightMiddleMotor{MiddleRight};
    WPI_TalonSRX m_rightBottomMotor{BottomRight};
    WPI_TalonSRX m_hopper{HopperID};
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




/* Hopper */
    // rev::CANSparkMax m_hopper{HopperID,
    //    rev::CANSparkMax::MotorType::kBrushless};

    frc::VictorSP test1 {0};  // left climb motor

    frc::VictorSP test2 {1};  // right climb motor

    /*instantiation of the compressor with its CAN ID and pneumatics*/
    // frc::Compressor compressor{PCM1};

    // frc::DoubleSolenoid climb{PCM1, 1, 6};  // 3 4

    // frc::DoubleSolenoid shooterHood{PCM1, 2, 5};

    // Sensor
    // frc::DigitalInput limitSwitch2{2};  // in the mag
    // frc::DigitalInput limitSwitch3{0};  // on the top of the mag

    //  frc::DigitalInput photoIN{0};
    //  frc::DigitalOutput photoOUT{1};

    /*SubSystem Objects  */
    // DriveTrain driveTrain{ m_leftTopMotor, m_rightTopMotor,
    //     m_leftMiddleMotor, m_rightMiddleMotor, m_leftBottomMotor,
    //     m_rightBottomMotor, Driver, LimeLight, ahrs };

    // Shooter shooter{m_leftShooter, m_rightShooter, shooterHood, Operator};

    // ClimbManager climber{test1, test2, Driver, climb};

    Hopper hopper{m_hopper, Operator};

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

    // PCM1 = 10,  //  GOOD

    HopperID = 9;  //  GOOD

    // LeftClimb = 13,  //  GOOD
    // RightClimb = 14;  //  GOOD

    double actualRPM;


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
