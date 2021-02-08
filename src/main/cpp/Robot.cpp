#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"
#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

double motorSpeed = 0.1;
int gearRatio = 6.9;
int robotPosL1 = 0;
int robotPosL2 = 0;
int robotPosL3 = 0;
int robotPosL4 = 0;
int robotPosL5 = 0;
int robotPosL6 = 0;
int robotPosL7 = 0;
int robotPosL8 = 0;

bool run_auto = true;

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
}

void Robot::RobotPeriodic() {
}
void Robot::AutonomousInit()    {
    shooter.InitPID();
    m_timer.Reset();
    m_timer.Start();
    ahrs.Reset();
    BottomLeftMotorEncoder->SetPosition(0);
    BottomRightMotorEncoder->SetPosition(0);

    //  automovement = new AutoMovement{*driveTrain.LeftMotors,
    //  *driveTrain.RightMotors,
    //  ahrs, *BottomLeftMotorEncoder,
    //  *BottomRightMotorEncoder};
}
void Robot::AutonomousPeriodic() {
    std::cout << ("Yaw:  \n");
    std::cout << (ahrs.GetYaw());
    // std::cout << ("\n Left Encoder \n");
    // std::cout << (BottomLeftMotorEncoder->GetPosition());
    std::cout << ("\n Right Encoder \n");
    std::cout << (BottomRightMotorEncoder->GetPosition());
    if (run_auto) {
        while (BottomLeftMotorEncoder->GetPosition() <= 20) {
            // forward 1
            driveTrain.LeftMotors->Set(motorSpeed);
        }
        while (abs(ahrs.GetYaw()) < 90) {
            // turn 1
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL1 = BottomLeftMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL1 + 15) {
            // forward 2
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }
        while (abs(ahrs.GetYaw()) < 135) {
            // turn 2
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL2 = BottomLeftMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL2 + 35) {
            // forward 3
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }
        while (abs(ahrs.GetYaw()) > 90) {
            // turn 3
            driveTrain.LeftMotors->Set(-motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
            robotPosL3 = BottomLeftMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL3 + 8) {
            // forward 4
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }
        while (abs(ahrs.GetYaw()) > 15) {
            // turn 4
            driveTrain.LeftMotors->Set(-motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
            robotPosL4 = BottomLeftMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL4 + 30) {
            // forward 5
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }
        while (abs(ahrs.GetYaw()) < 90) {
            // turn 5
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL5 = BottomLeftMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL5 + 8) {
            // forward 6
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }
        while (abs(ahrs.GetYaw()) < 135) {
            // turn 6
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL6 = BottomLeftMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL6 + 35) {
            // forward 7
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }
        while (abs(ahrs.GetYaw()) > 90) {
            // turn 7
            driveTrain.LeftMotors->Set(-motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
            robotPosL7 = BottomLeftMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL7 + 8) {
            // forward 8
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }
        // this probably wont work
        while (abs(ahrs.GetYaw()) > 0) {
            // turn 8
            driveTrain.LeftMotors->Set(-motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
            robotPosL8 = BottomLeftMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL8 + 20) {
            // forward 9
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }

        // stop motors
        driveTrain.LeftMotors->Set(0);
        driveTrain.RightMotors->Set(0);
        run_auto = false;
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
