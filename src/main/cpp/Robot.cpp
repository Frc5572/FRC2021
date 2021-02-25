#include <iostream>
#include <string>
#include <math.h>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"
#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

double motorSpeed = 0.1;
double gearRatio = 10;
double wRotationFoot = 12 * gearRatio / 18.85;
bool run_auto = true;
double rotPFT = 12* gearRatio / 18.85;
int robotPosL;
int robotPosR;
int robotPosL2;
int robotPosR2;
int robotPosL3;
int robotPosR3;
int robotPosL4;
int robotPosR4;
int robotPosL5;
int robotPosR5;
int robotPosL6;
int robotPosR6;
int robotPosL7;
int robotPosR7;
int robotPosL8;
int robotPosR8;
int robotPosL9;
int robotPosR9;
int robotPosL10;
int robotPosR10;
int robotPosL11;
int robotPosR11;
int robotPosL12;
int robotPosR12;
int robotPosL13;
int robotPosR13;
int robotPosL14;
int robotPosR14;
int robotPosL15;
int robotPosR15;
int robotPosL16;
int robotPosR16;
int robotPosL17;
int robotPosR17;
int robotPosL18;
int robotPosR18;
int robotPosL19;
int robotPosR19;
int robotPosL20;
int robotPosR20;
int robotPosL21;
int robotPosR21;
int robotPosL22;
int robotPosR22;
int robotPosL23;
int robotPosR23;
int robotPosL24;
int robotPosR24;
int robotPosL25;
int robotPosR25;

/*
 * IMPORTANT REMINDER
 * not all robot turns are right angles
 * cannot use pythagorean theorem on all angles
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * need to use law of cosines
*/

void Robot::RobotInit() {
    driveTrain.RightMotors->SetInverted(true);
    m_timer.Start();
    m_leftBottomMotor.RestoreFactoryDefaults();
    m_leftMiddleMotor.RestoreFactoryDefaults();
    m_leftTopMotor.RestoreFactoryDefaults();
    m_rightBottomMotor.RestoreFactoryDefaults();
    m_rightMiddleMotor.RestoreFactoryDefaults();
    m_rightTopMotor.RestoreFactoryDefaults();
    m_leftBottomMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_leftMiddleMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_leftTopMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightBottomMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightMiddleMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightTopMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    frc::SmartDashboard::PutString("Path", "Path A");
}

void Robot::RobotPeriodic() {
}
void Robot::AutonomousInit()    {
    shooter.InitPID();
    m_timer.Reset();
    m_timer.Start();
    ahrs.Reset();
    MiddleLeftMotorEncoder->SetPosition(0);
    MiddleRightMotorEncoder->SetPosition(0);

    //  automovement = new AutoMovement{*driveTrain.LeftMotors,
    //  *driveTrain.RightMotors,
    //  ahrs, *BottomLeftMotorEncoder,
    //  *BottomRightMotorEncoder};
}
void Robot::AutonomousPeriodic() {
    MiddleLeftMotorEncoder->SetPosition(0);
    MiddleRightMotorEncoder->SetPosition(0);
    std::cout << ("Yaw:  \n");
    std::cout << (ahrs.GetYaw());
    // std::cout << ("\n Left Encoder \n");
    // std::cout << (BottomLeftMotorEncoder->GetPosition());
    std::cout << ("\n Right Encoder \n");
    std::cout << (MiddleRightMotorEncoder->GetPosition());
    if (run_auto) {
        auto pathName = frc::SmartDashboard::GetString("Path", "Path A");
        if (pathName == "Path A") {
            if (run_auto) {
                // D1 -> B1
                while (MiddleLeftMotorEncoder->GetPosition() <= 5 * rotPFT && MiddleRightMotorEncoder->GetPosition() < 5 * rotPFT) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }
                // B1 turn
                while (abs(ahrs.GetYaw()) < 116.565) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(-.1);
                    robotPosL = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR = MiddleRightMotorEncoder->GetPosition();
                }
                // B1 -> C3
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL +(5.59 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR + (5.59 * rotPFT)) {
                        driveTrain.LeftMotors->Set(.1);
                        driveTrain.RightMotors->Set(.1);
                }
                // C3 -> D5
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL2 +(5.59 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR2 + (5.59 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }
                // D5 turn
                while (abs(ahrs.GetYaw()) < 135) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(-.1);
                    robotPosL3 = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR3 = MiddleRightMotorEncoder->GetPosition();
                }
                // D5 -> E6
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL3 +(3.54 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR3 + (3.54 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }
                // E6 turn
                while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                    driveTrain.LeftMotors->Set(-.1);
                    driveTrain.RightMotors->Set(.1);
                    robotPosL5 = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR5 = MiddleRightMotorEncoder->GetPosition();
                }
                // E6 -> A6
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL5 +(8 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR5 + (8 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }
                // A6 turn
                while (abs(ahrs.GetYaw()) < 135) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(-.1);
                    robotPosL6 = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR6 = MiddleRightMotorEncoder->GetPosition();
                }
                // A6 -> B7
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL6 +(3.54 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR6 + (3.54 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }

                // B7 turn
                while (abs(ahrs.GetYaw()) > 116.565) {
                    driveTrain.LeftMotors->Set(-.1);
                    driveTrain.RightMotors->Set(.1);
                    robotPosL7 = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR7 = MiddleRightMotorEncoder->GetPosition();
                }
                // B7 -> C9
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL7 +(5.59 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR7 + (5.59 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }
                // C9 -> D11
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL8 +(5.59 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR8 + (5.59 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }
                // D11 turn
                while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                    driveTrain.LeftMotors->Set(-.1);
                    driveTrain.RightMotors->Set(.1);
                    robotPosL9 = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR9 = MiddleRightMotorEncoder->GetPosition();
                }
                // D11 -> B11
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL9 + (5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR9 + (5 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }

                // STOP
                driveTrain.LeftMotors->Set(0);
                driveTrain.RightMotors->Set(0);

                run_auto = false;
            }
        } else if (pathName == "Path B") {
            if (run_auto) {
                while (MiddleLeftMotorEncoder->GetPosition() <= wRotationFoot * 5) {
                    // forward 1
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                while (abs(ahrs.GetYaw()) < 90) {
                    // turn 1
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(-motorSpeed);
                    robotPosL = MiddleLeftMotorEncoder->GetPosition();
                }
                // calculate
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL + wRotationFoot * 5) {
                    // forward 2
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                while (abs(ahrs.GetYaw()) < 135) {
                    // turn 2
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(-motorSpeed);
                    robotPosL2 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL2 + wRotationFoot *  sqrt(50)) {
                    // forward 3
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                while (abs(ahrs.GetYaw()) > 90) {
                    // turn 3
                    driveTrain.LeftMotors->Set(-motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                    robotPosL3 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL3 + wRotationFoot * 2.5) {
                    // forward 4
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                while (abs(ahrs.GetYaw()) > 22.5) {
                    // turn 4
                    driveTrain.LeftMotors->Set(-motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                    robotPosL4 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL4 + wRotationFoot * sqrt(31.25)) {
                    // forward 5
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                while (abs(ahrs.GetYaw()) < 90) {
                    // turn 5
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(-motorSpeed);
                    robotPosL5 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL5 + wRotationFoot * 2.5) {
                    // forward 6
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                while (abs(ahrs.GetYaw()) < 135) {
                    // turn 6
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(-motorSpeed);
                    robotPosL6 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL6 + wRotationFoot * sqrt(50)) {
                    // forward 7
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                while (abs(ahrs.GetYaw()) > 90) {
                    // turn 7
                    driveTrain.LeftMotors->Set(-motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                    robotPosL7 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL7 + wRotationFoot * 2.5) {
                    // forward 8
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                // this probably wont work
                while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                    // turn 8
                    driveTrain.LeftMotors->Set(-motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                    robotPosL8 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL8 + wRotationFoot * 5) {
                    // forward 9
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }

                // stop motors
                driveTrain.LeftMotors->Set(0);
                driveTrain.RightMotors->Set(0);
                run_auto = false;
            }
        } else if (pathName == "Path B") {
            while (MiddleLeftMotorEncoder->GetPosition() <= wRotationFoot * 5) {
                // forward 1
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) < 90) {
                // turn 1
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(-motorSpeed);
                robotPosL = MiddleLeftMotorEncoder->GetPosition();
            }
            // calculate
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL + wRotationFoot * 5) {
                // forward 2
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) < 135) {
                // turn 2
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(-motorSpeed);
                robotPosL2 = MiddleLeftMotorEncoder->GetPosition();
            }
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL2 + wRotationFoot *  sqrt(50)) {
                // forward 3
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) > 90) {
                // turn 3
                driveTrain.LeftMotors->Set(-motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
                robotPosL3 = MiddleLeftMotorEncoder->GetPosition();
            }
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL3 + wRotationFoot * 2.5) {
                // forward 4
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) > 22.5) {
                // turn 4
                driveTrain.LeftMotors->Set(-motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
                robotPosL4 = MiddleLeftMotorEncoder->GetPosition();
            }
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL4 + wRotationFoot * sqrt(31.25)) {
                // forward 5
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) < 90) {
                // turn 5
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(-motorSpeed);
                robotPosL5 = MiddleLeftMotorEncoder->GetPosition();
            }
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL5 + wRotationFoot * 2.5) {
                // forward 6
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) < 135) {
                // turn 6
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(-motorSpeed);
                robotPosL6 = MiddleLeftMotorEncoder->GetPosition();
            }
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL6 + wRotationFoot * sqrt(50)) {
                // forward 7
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) > 90) {
                // turn 7
                driveTrain.LeftMotors->Set(-motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
                robotPosL7 = MiddleLeftMotorEncoder->GetPosition();
            }
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL7 + wRotationFoot * 2.5) {
                // forward 8
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            // this probably wont work
            while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                // turn 8
                driveTrain.LeftMotors->Set(-motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
                robotPosL8 = MiddleLeftMotorEncoder->GetPosition();
            }
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL8 + wRotationFoot * 5) {
                // forward 9
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }

            // stop motors
            driveTrain.LeftMotors->Set(0);
            driveTrain.RightMotors->Set(0);
            run_auto = false;
        }else if (pathName == "Path C") {
            // work in progress
        } else if (pathName == "Path D") {
            // work in progress
        } else if (pathName == "Path E") {
            //Bounce Path
            if (run_auto) {
                // D1 -> B1
                while (MiddleLeftMotorEncoder->GetPosition() <= 2.5 * rotPFT && MiddleRightMotorEncoder->GetPosition() < 2.5 * rotPFT) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }
                // B1 turn
                while (abs(ahrs.GetYaw()) < 63.435) {
                    driveTrain.LeftMotors->Set(-.1);
                    driveTrain.RightMotors->Set(.1);
                    robotPosL = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR = MiddleRightMotorEncoder->GetPosition();
                }
                // B1 -> C3
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL +(5.59 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR + (5.59 * rotPFT)) {
                        driveTrain.LeftMotors->Set(.1);
                        driveTrain.RightMotors->Set(.1);
                }
                // D5 turn 1
                while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(-.1);
                }
                // D5 turn 2
                while (abs(ahrs.GetYaw()) < 74.055) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(-.1);
                    robotPosL2 = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR2 = MiddleRightMotorEncoder->GetPosition();
                }
                // D5 -> E6
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL2 +(7.9058 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR2 + (7.9058 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }
                // B1 turn
                while (abs(ahrs.GetYaw()) > 47.49) {
                    driveTrain.LeftMotors->Set(-.1);
                    driveTrain.RightMotors->Set(.1);
                    robotPosL3 = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR3 = MiddleRightMotorEncoder->GetPosition();
                }
                // E6 -> A6
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL3 +(3.5355 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR3 + (3.5355 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }
                // D5 turn 1
                while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                    driveTrain.LeftMotors->Set(-.1);
                    driveTrain.RightMotors->Set(.1);

                }
                // D5 turn 2
                while (abs(ahrs.GetYaw()) < 71.565) {
                    driveTrain.LeftMotors->Set(-.1);
                    driveTrain.RightMotors->Set(.1);
                    robotPosL4 = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR4 = MiddleRightMotorEncoder->GetPosition();
                }
                // A6 -> B7
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL4 +(10.3078 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR4 + (10.3078 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }
                // D5 turn 1
                while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(-.1);

                }
                // D5 turn 2
                while (abs(ahrs.GetYaw()) < 71.565) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(-.1);
                    robotPosL5 = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR5 = MiddleRightMotorEncoder->GetPosition();
                }
                // B7 -> C9
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL5 +(10.3078 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR5 + (10.3078 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }
                // D11 turn
                while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                    driveTrain.LeftMotors->Set(-.1);
                    driveTrain.RightMotors->Set(.1);
                    robotPosL6 = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR6 = MiddleRightMotorEncoder->GetPosition();
                }
                // D7 -> D8
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL6 + (2.5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR6 + (2.5 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }
                // D5 turn 2
                while (abs(ahrs.GetYaw()) < 71.565) {
                    driveTrain.LeftMotors->Set(-.1);
                    driveTrain.RightMotors->Set(.1);
                    robotPosL7 = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR7 = MiddleRightMotorEncoder->GetPosition();
                }
                // B7 -> C9
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL7 +(10.3078 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR7 + (10.3078 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }
                // D11 turn
                while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(-.1);
                }
                // B7 turn
                while (abs(ahrs.GetYaw()) < 56.31) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(-.1);
                    robotPosL8 = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR8 = MiddleRightMotorEncoder->GetPosition();
                }
                // B7 -> C9
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL8 +(5.9 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR8 + (5.9 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }
                // D11 turn
                while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                    driveTrain.LeftMotors->Set(-.1);
                    driveTrain.RightMotors->Set(.1);
                    robotPosL10 = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR10 = MiddleRightMotorEncoder->GetPosition();
                }
                // D7 -> D8
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL10 + (2.5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR10 + (2.5 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }

                // STOP
                driveTrain.LeftMotors->Set(0);
                driveTrain.RightMotors->Set(0);

                run_auto = false;
            }
        } else if (pathName == "Path F") {
            // work in progress
        }
    }
}



void Robot::TeleopInit() {
    shooter.InitPID();
}

void Robot::TeleopPeriodic() {
    //  LimeLight.Update();

    // shooter.RunPID();

    driveTrain.Drive();

    // shooter.Shots();

    // climber.ClimbPeriodic();

    // hopper.HopperPeriodic();
}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif
