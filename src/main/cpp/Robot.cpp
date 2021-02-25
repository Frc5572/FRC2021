#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"
#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

double motorSpeed = 0.1;
double gearRatio = 10;
double wRotationFoot = 12 * gearRatio / 18.85;
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
    BottomLeftMotorEncoder->SetPosition(0);
    BottomRightMotorEncoder->SetPosition(0);
    std::cout << ("Yaw:  \n");
    std::cout << (ahrs.GetYaw());
    // std::cout << ("\n Left Encoder \n");
    // std::cout << (BottomLeftMotorEncoder->GetPosition());
    std::cout << ("\n Right Encoder \n");
    std::cout << (BottomRightMotorEncoder->GetPosition());
    if (run_auto) {
        while (BottomLeftMotorEncoder->GetPosition() <= wRotationFoot * 9.35) {
            // forward 9.35 feet
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }
        while (abs(ahrs.GetYaw()) < 90) {
            // turn right 90
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL1 = BottomLeftMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL1 + wRotationFoot * 2.7) {
            // forward 2.7 feet
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }
        while (abs(ahrs.GetYaw()) < 180 && ahrs.GetYaw() < 0) {
            // turn right 90
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL1 = BottomLeftMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL1 + wRotationFoot * 3.35) {
            // forward 3.3 feet
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }
        while (abs(ahrs.GetYaw()) > 90) {
            // turn right 90
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL1 = BottomLeftMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL1 + wRotationFoot * 2.7) {
            // forward 2.7 feet
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }
        while (abs(ahrs.GetYaw()) > 11.77) {
            // turn right 78.23
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL1 = BottomLeftMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL1 + wRotationFoot * 12.25) {
            // forward 12.25 feet
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        } 
        while (abs(ahrs.GetYaw()) > 101.77) {
            // turn left 90
            driveTrain.LeftMotors->Set(-motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
            robotPosL1 = BottomLeftMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL1 + wRotationFoot * 3) {
            // forward 3 feet
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        } 
        while (abs(ahrs.GetYaw()) < 11.77) {
            // turn left 90
            driveTrain.LeftMotors->Set(-motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
            robotPosL1 = BottomLeftMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL1 + wRotationFoot * 5.475) {
            // forward 5.475 feet
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        } 
        while (abs(ahrs.GetYaw()) > 101.77) {
            // turn left 90
            driveTrain.LeftMotors->Set(-motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
            robotPosL1 = BottomLeftMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL1 + wRotationFoot * 3) {
            // forward 3 feet
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        } 
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
