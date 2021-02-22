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
        for (int i = 0; i < 2; i++) {
            // B1.5 turn
            while (abs(ahrs.GetYaw()) < 71.565) {
                driveTrain.LeftMotors->Set(-.1);
                driveTrain.RightMotors->Set(.1);
                robotPosL = MiddleLeftMotorEncoder->GetPosition();
                robotPosR = MiddleRightMotorEncoder->GetPosition();
            }
            // B1.5 -> A2
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL +(3.95* rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR + (3.95 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
            }
            // A2 Turn
            while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(-.1);
                robotPosL2 = MiddleLeftMotorEncoder->GetPosition();
                robotPosR2 = MiddleRightMotorEncoder->GetPosition();
            }
            // A2 -> A4
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL2 +(5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR2 + (5 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
            }
            // A4 turn
            while (abs(ahrs.GetYaw()) < 45) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(-.1);
                robotPosL3 = MiddleLeftMotorEncoder->GetPosition();
                robotPosR3 = MiddleRightMotorEncoder->GetPosition();
            }
            // A4 -> C6
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL3 +(7 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR3 + (7 * rotPFT)) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(.1);
            }
            // C6 Turn
            while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                driveTrain.LeftMotors->Set(-.1);
                driveTrain.RightMotors->Set(.1);
                robotPosL4 = MiddleLeftMotorEncoder->GetPosition();
                robotPosR4 = MiddleRightMotorEncoder->GetPosition();
            }
            // C6 -> C7
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL4 +(2.5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR4 + (2.5 * rotPFT)) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(.1);
            }
            // C7 turn
            while (abs(ahrs.GetYaw()) < 45) {
                driveTrain.LeftMotors->Set(-.1);
                driveTrain.RightMotors->Set(.1);
                robotPosL5 = MiddleLeftMotorEncoder->GetPosition();
                robotPosR5 = MiddleRightMotorEncoder->GetPosition();
            }
            // C7 -> A9
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL5 +(7 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR5 + (7 * rotPFT)) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(.1);
            }
            // A9 turn 1
            while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(-.1);
            }
            // A9 turn 2
            while (abs(ahrs.GetYaw()) < 45) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(-.1);
                robotPosL6 = MiddleLeftMotorEncoder->GetPosition();
                robotPosR6 = MiddleRightMotorEncoder->GetPosition();
            }
            // A9 -> C11
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL6 +(7 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR6 + (7 * rotPFT)) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(.1);
            }
            // C11 turn
            while (abs(ahrs.GetYaw()) < 90) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(-.1);
                robotPosL7 = MiddleLeftMotorEncoder->GetPosition();
                robotPosR7 = MiddleRightMotorEncoder->GetPosition();
            }
            // C11 -> E11
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL7 + (5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR7 + (5 * rotPFT)) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(.1);
            }
            // E11 turn
            while (abs(ahrs.GetYaw()) < 180) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(-.1);
                robotPosL8 = MiddleLeftMotorEncoder->GetPosition();
                robotPosR8 = MiddleRightMotorEncoder->GetPosition();
            }
            // E11 -> E6
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL8 +(12.5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR8 + (12.5 * rotPFT)) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(.1);
            }
            // E6 turn
            while (abs(ahrs.GetYaw()) > 135) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(-.1);
                robotPosL9 = MiddleLeftMotorEncoder->GetPosition();
                robotPosR9 = MiddleRightMotorEncoder->GetPosition();
            }
            // E6 -> C4
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL9 +(7 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR9 + (7 * rotPFT)) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(.1);
            }
            // C4 turn
            while (abs(ahrs.GetYaw()) < 180) {
                driveTrain.LeftMotors->Set(-.1);
                driveTrain.RightMotors->Set(.1);
                robotPosL10 = MiddleLeftMotorEncoder->GetPosition();
                robotPosR10 = MiddleRightMotorEncoder->GetPosition();
            }
            // C4 -> C3
            while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL10 + (2.5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR10 + (2.5 * rotPFT)) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(.1);
            }
            if( i == 0){
                // C3 turn
                while (abs(ahrs.GetYaw()) > 135) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(-.1);
                    robotPosL11 = MiddleLeftMotorEncoder->GetPosition();
                    robotPosR11 = MiddleRightMotorEncoder->GetPosition();
                }
                // C3 -> B2
                while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL11 + (3.5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR11 + (3.5 * rotPFT)) {
                    driveTrain.LeftMotors->Set(.1);
                    driveTrain.RightMotors->Set(.1);
                }
            } else {
                continue;
            }
        }

        // C3 turn 2.0
        while (abs(ahrs.GetYaw()) > 135) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL12 = MiddleLeftMotorEncoder->GetPosition();
            robotPosR12 = MiddleRightMotorEncoder->GetPosition();
        }
        // C3 -> D2
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL12 + (3.5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR12 + (3.5 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // D2 turn
        while (abs(ahrs.GetYaw()) > 90) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL13 = MiddleLeftMotorEncoder->GetPosition();
            robotPosR13 = MiddleRightMotorEncoder->GetPosition();
        }
        // D2 -> E2
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL13 + (2.5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR13 + (2.5 * rotPFT)) {
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
