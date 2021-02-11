#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"
#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

int gearRatio = 10;
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
bool runIf = true;

void Robot::RobotInit() {
    driveTrain.RightMotors->SetInverted(true);
    m_timer.Start();
    m_leftBottomMotor.RestoreFactoryDefaults();
    m_leftMiddleMotor.RestoreFactoryDefaults();
    m_leftTopMotor.RestoreFactoryDefaults();
    m_rightBottomMotor.RestoreFactoryDefaults();
    m_rightMiddleMotor.RestoreFactoryDefaults();
    m_rightTopMotor.RestoreFactoryDefaults();
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
    std::cout << ("Yaw:  \n");
    std::cout << (ahrs.GetYaw());
    // std::cout << ("\n Left Encoder \n");
    // std::cout << (BottomLeftMotorEncoder->GetPosition());
    // std::cout << ("\n Right Encoder \n");
    // std::cout << (BottomRightMotorEncoder->GetPosition());
    if (runIf) {
        // D1 -> B1
        while (MiddleLeftMotorEncoder->GetPosition() <= 2.5 * rotPFT && MiddleRightMotorEncoder->GetPosition() < 2.5 * rotPFT) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // B1 turn
        while (ahrs.GetYaw() > -116.565) {
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
        // D5 turn
        while (ahrs.GetYaw() < 45) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
            robotPosL3 = MiddleLeftMotorEncoder->GetPosition();
            robotPosR3 = MiddleRightMotorEncoder->GetPosition();
        }
        // D5 -> E6
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL3 +(7.9058 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR3 + (7.9058 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // B1 turn
        while (ahrs.GetYaw() > 26.565 && ahrs.GetYaw() <= 45) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL = MiddleLeftMotorEncoder->GetPosition();
            robotPosR = MiddleRightMotorEncoder->GetPosition();
        }
        // E6 -> A6
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL5 +(3.5355 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR5 + (3.5355 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // A6 turn
        while (ahrs.GetYaw() < -77.471) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL6 = MiddleLeftMotorEncoder->GetPosition();
            robotPosR6 = MiddleRightMotorEncoder->GetPosition();
        }
        // A6 -> B7
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL6 +(10.3078 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR6 + (10.3078 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // B7 turn
        while (abs(ahrs.GetYaw()) > 88.493) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
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
            robotPosL9 = MiddleLeftMotorEncoder->GetPosition();
            robotPosR9 = MiddleRightMotorEncoder->GetPosition();
        }
        // D7 -> D8
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL9 + (2.5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR9 + (2.5 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // B7 turn
        while (abs(ahrs.GetYaw()) < -104.036) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL10 = MiddleLeftMotorEncoder->GetPosition();
            robotPosR10 = MiddleRightMotorEncoder->GetPosition();
        }
        // B7 -> C9
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL10 +(10.3078 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR10 + (10.3078 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // B7 turn
        while (abs(ahrs.GetYaw()) > 153.435) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
            robotPosL11 = MiddleLeftMotorEncoder->GetPosition();
            robotPosR11 = MiddleRightMotorEncoder->GetPosition();
        }
        // B7 -> C9
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL11 +(67.08204 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR11 + (67.08204 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // D11 turn
        while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL12 = MiddleLeftMotorEncoder->GetPosition();
            robotPosR12 = MiddleRightMotorEncoder->GetPosition();
        }
        // D7 -> D8
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL12 + (2.5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR12 + (2.5 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }

        // STOP
        driveTrain.LeftMotors->Set(0);
        driveTrain.RightMotors->Set(0);

        runIf = false;
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
