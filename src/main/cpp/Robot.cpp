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
