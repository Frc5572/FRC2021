#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"
#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

double gearRatio = 10;
int rotPFT = 12* gearRatio / 18.85;
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
    // std::cout << ("\n Right Encoder \n");
    // std::cout << (BottomRightMotorEncoder->GetPosition());
    if (runIf) {
        // D1 -> B1
        while (BottomLeftMotorEncoder->GetPosition() <= 5 * rotPFT && BottomRightMotorEncoder->GetPosition() < 5 * rotPFT) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // B1 turn
        while (abs(ahrs.GetYaw()) < 117.565) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
            robotPosL = BottomLeftMotorEncoder->GetPosition();
            robotPosR = BottomRightMotorEncoder->GetPosition();
        }
        // B1 -> C3
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL +(5.59 * rotPFT) && BottomRightMotorEncoder->GetPosition() < robotPosR + (5.59 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL2 = BottomLeftMotorEncoder->GetPosition();
            robotPosR2 = BottomRightMotorEncoder->GetPosition();
        }
        // C3 -> D5
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL2 +(5.59 * rotPFT) && BottomRightMotorEncoder->GetPosition() < robotPosR2 + (5.59 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // D5 turn
        while (abs(ahrs.GetYaw()) < 135) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
            robotPosL3 = BottomLeftMotorEncoder->GetPosition();
            robotPosR3 = BottomRightMotorEncoder->GetPosition();
        }
        // D5 -> E6
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL3 +(3.54 * rotPFT) && BottomRightMotorEncoder->GetPosition() < robotPosR3 + (3.54 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL4 = BottomLeftMotorEncoder->GetPosition();
            robotPosR4 = BottomRightMotorEncoder->GetPosition();
        }
        // E6 turn
        while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL5 = BottomLeftMotorEncoder->GetPosition();
            robotPosR5 = BottomRightMotorEncoder->GetPosition();
        }
        // E6 -> A6
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL5 +(10 * rotPFT) && BottomRightMotorEncoder->GetPosition() < robotPosR5 + (10 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // A6 turn
        while (abs(ahrs.GetYaw()) < 135) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL6 = BottomLeftMotorEncoder->GetPosition();
            robotPosR6 = BottomRightMotorEncoder->GetPosition();
        }
        // A6 -> B7
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL6 +(3.54 * rotPFT) && BottomRightMotorEncoder->GetPosition() < robotPosR6 + (3.54 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // B7 turn
        while (abs(ahrs.GetYaw()) > 117.565) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL7 = BottomLeftMotorEncoder->GetPosition();
            robotPosR7 = BottomRightMotorEncoder->GetPosition();
        }
        // B7 -> C9
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL7 +(5.59 * rotPFT) && BottomRightMotorEncoder->GetPosition() < robotPosR7 + (5.59 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL8 = BottomLeftMotorEncoder->GetPosition();
            robotPosR8 = BottomRightMotorEncoder->GetPosition();
        }
        // C9 -> D11
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL8 +(5.59 * rotPFT) && BottomRightMotorEncoder->GetPosition() < robotPosR8 + (5.59 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // D11 turn
        while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL9 = BottomLeftMotorEncoder->GetPosition();
            robotPosR9 = BottomRightMotorEncoder->GetPosition();
        }
        // D11 -> B11
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL9 + (5 * rotPFT) && BottomRightMotorEncoder->GetPosition() < robotPosR9 + (5 * rotPFT)) {
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
