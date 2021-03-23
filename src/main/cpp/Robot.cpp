#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"
#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

bool blueRed = true;
double motorSpeed = 0.1;
int motorRotation = 2048;
double wheelRotation = motorRotation * 8.5;
double rotPFT = wheelRotation * 0.75;
int robotPosL1 = 0;
int robotPosL2 = 0;
int robotPosL3 = 0;
int robotPosL4 = 0;
int robotPosL5 = 0;
int robotPosL6 = 0;
int robotPosL7 = 0;
int robotPosL8 = 0;

bool runAuto = true;

void Robot::RobotInit() {
    driveTrain.RightMotors->SetInverted(true);
    m_timer.Start();
    frc::SmartDashboard::PutString("Path", "Path A");
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
}

void Robot::AutonomousPeriodic() {
    MiddleLeftMotorEncoder->SetPosition(0);
    MiddleRightMotorEncoder->SetPosition(0);
    if (runAuto) {
        auto pathName = frc::SmartDashboard::GetString("Path", "Path B");
        if (pathName == "Path A") {
            // path A
        } else if (pathName == "Path B") {
            if (blueRed == false) {
                while (abs(ahrs.GetYaw()) < 26.57) {
                    // turn 1
                    driveTrain.LeftMotors->Set(-motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                    robotPosL1 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL1 + rotPFT * 5.6) {
                    // forward 1
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                while (abs(ahrs.GetYaw()) < 45) {
                    // turn 2
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(-motorSpeed);
                    robotPosL2 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL2 + rotPFT * sqrt(50)) {
                    // forward 2
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                    // turn 4
                    driveTrain.LeftMotors->Set(-motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                    robotPosL4 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (abs(ahrs.GetYaw()) < 45) {
                    // turn 3
                    driveTrain.LeftMotors->Set(-motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                    robotPosL3 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL3 + rotPFT *  sqrt(50)) {
                    // forward 3
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() > 0) {
                    // turn 4
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(-motorSpeed);
                    robotPosL4 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (abs(ahrs.GetYaw()) < 5.7) {
                    // turn 4
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(-motorSpeed);
                    robotPosL4 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL4 + rotPFT * 11) {
                    // forward 3
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                // stop motors
                driveTrain.LeftMotors->Set(0);
                driveTrain.RightMotors->Set(0);
                runAuto = false;
            } else if (blueRed == true) {
                while (abs(ahrs.GetYaw()) < 11.31) {
                    // turn 1
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(-motorSpeed);
                    robotPosL1 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL1 + rotPFT *  sqrt(181.25)) {
                    // forward 2
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                    // turn 4
                    driveTrain.LeftMotors->Set(-motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                    robotPosL4 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (abs(ahrs.GetYaw()) < 45) {
                    // turn 2
                    driveTrain.LeftMotors->Set(-motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                    robotPosL2 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL2 + rotPFT * sqrt(50)) {
                    // forward 3
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                while (abs(ahrs.GetYaw()) < 45) {
                    // turn 3
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(-motorSpeed);
                    robotPosL3 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL3 + rotPFT * sqrt(50)) {
                    // forward 4
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                    // turn 4
                    driveTrain.LeftMotors->Set(-motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                    robotPosL4 = MiddleLeftMotorEncoder->GetPosition();
                }
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL4 + rotPFT * 2.5) {
                    // forward 5
                    driveTrain.LeftMotors->Set(motorSpeed);
                    driveTrain.RightMotors->Set(motorSpeed);
                }
                // stop motors
                driveTrain.LeftMotors->Set(0);
                driveTrain.RightMotors->Set(0);
                runAuto = false;
            }
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
