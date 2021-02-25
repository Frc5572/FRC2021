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
        // 1
        while (MiddleLeftMotorEncoder->GetPosition() <= 5 * rotPFT && MiddleRightMotorEncoder->GetPosition() < 5 * rotPFT) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // 1 turn
        while (abs(ahrs.GetYaw()) < 90) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL = MiddleLeftMotorEncoder->GetPosition();
            robotPosR = MiddleRightMotorEncoder->GetPosition();
        }
        // 2
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL +(5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR + (5 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // 2 turns
        //ahrs.ZeroYaw();
        //added
        /*while (abs(ahrs.GetYaw()) > 90) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
            robotPosL = MiddleLeftMotorEncoder->GetPosition();
            robotPosR = MiddleRightMotorEncoder->GetPosition();
        }*/
        while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() > 0) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
            robotPosL2 = MiddleLeftMotorEncoder->GetPosition();
            robotPosR2 = MiddleRightMotorEncoder->GetPosition();
        }
        // // 3
        // while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL2 +(12.5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR2 + (12.5 * rotPFT)) {
        //     driveTrain.LeftMotors->Set(.1);
        //     driveTrain.RightMotors->Set(.1);
        // }
        // // 3 turn
        // while (abs(ahrs.GetYaw()) < 90) {
        //     driveTrain.LeftMotors->Set(.1);
        //     driveTrain.RightMotors->Set(-.1);
        //     robotPosL3 = MiddleLeftMotorEncoder->GetPosition();
        //     robotPosR3 = MiddleRightMotorEncoder->GetPosition();
        // }
        // // 4
        // while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL3 +(5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR3 + (5 * rotPFT)) {
        //     driveTrain.LeftMotors->Set(.1);
        //     driveTrain.RightMotors->Set(.1);
        // }
        // // 4 turn
        // while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
        //     driveTrain.LeftMotors->Set(-.1);
        //     driveTrain.RightMotors->Set(.1);
        //     robotPosL4 = MiddleLeftMotorEncoder->GetPosition();
        //     robotPosR4 = MiddleRightMotorEncoder->GetPosition();
        // }
        // // 5
        // while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL4 +(5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR4 + (5 * rotPFT)) {
        //     driveTrain.LeftMotors->Set(.1);
        //     driveTrain.RightMotors->Set(.1);
        // }
        // // 5 turn
        // while (abs(ahrs.GetYaw()) < 90) {
        //     driveTrain.LeftMotors->Set(-.1);
        //     driveTrain.RightMotors->Set(.1);
        //     robotPosL5 = MiddleLeftMotorEncoder->GetPosition();
        //     robotPosR5 = MiddleRightMotorEncoder->GetPosition();
        // }
        // // 6
        // while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL5 +(5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR5 + (5 * rotPFT)) {
        //     driveTrain.LeftMotors->Set(.1);
        //     driveTrain.RightMotors->Set(.1);
        // }
        // // 6 turn
        // while (abs(ahrs.GetYaw()) < 180 && ahrs.GetYaw() > 0) {
        //     driveTrain.LeftMotors->Set(-.1);
        //     driveTrain.RightMotors->Set(.1);
        //     robotPosL6 = MiddleLeftMotorEncoder->GetPosition();
        //     robotPosR6 = MiddleRightMotorEncoder->GetPosition();
        // }
        // // 7
        // while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL6 + (5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR6 + (5 * rotPFT)) {
        //     driveTrain.LeftMotors->Set(.1);
        //     driveTrain.RightMotors->Set(.1);
        // }
        // // 7 turn
        // while (abs(ahrs.GetYaw()) < 90) {
        //     driveTrain.LeftMotors->Set(-.1);
        //     driveTrain.RightMotors->Set(.1);
        //     robotPosL7 = MiddleLeftMotorEncoder->GetPosition();
        //     robotPosR7 = MiddleRightMotorEncoder->GetPosition();
        // }
        // // 8
        // while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL7 +(5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR7 + (5 * rotPFT)) {
        //     driveTrain.LeftMotors->Set(.1);
        //     driveTrain.RightMotors->Set(.1);
        // }
        // // 8 turn
        // while (abs(ahrs.GetYaw()) < 180 && ahrs.GetYaw() ) {
        //     driveTrain.LeftMotors->Set(-.1);
        //     driveTrain.RightMotors->Set(.1);
        //     robotPosL8 = MiddleLeftMotorEncoder->GetPosition();
        //     robotPosR8 = MiddleRightMotorEncoder->GetPosition();
        // }
        // // 9
        // while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL8 +(12.5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR8 + (12.5 * rotPFT)) {
        //     driveTrain.LeftMotors->Set(.1);
        //     driveTrain.RightMotors->Set(.1);
        // }
        // // 9 turn
        // while (abs(ahrs.GetYaw()) < 90) {
        //     driveTrain.LeftMotors->Set(.1);
        //     driveTrain.RightMotors->Set(-.1);
        //     robotPosL9 = MiddleLeftMotorEncoder->GetPosition();
        //     robotPosR9 = MiddleRightMotorEncoder->GetPosition();
        // }
        // // 10
        // while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL9 + (5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR9 + (5 * rotPFT)) {
        //     driveTrain.LeftMotors->Set(.1);
        //     driveTrain.RightMotors->Set(.1);
        // }
        // // 10 turn
        // while (abs(ahrs.GetYaw()) < 180) {
        //     driveTrain.LeftMotors->Set(-.1);
        //     driveTrain.RightMotors->Set(.1);
        //     robotPosL10 = MiddleLeftMotorEncoder->GetPosition();
        //     robotPosR10 = MiddleRightMotorEncoder->GetPosition();
        // }
        // // 11
        // while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL10 + (5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR10 + (5 * rotPFT)) {
        //     driveTrain.LeftMotors->Set(.1);
        //     driveTrain.RightMotors->Set(.1);
        // }

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
