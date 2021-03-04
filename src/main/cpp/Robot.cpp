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

int runIf = 1;

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
    // red
    if (runIf == 1) {
        // C1 -> C3
        while (MiddleLeftMotorEncoder->GetPosition() <= 5 * rotPFT && MiddleRightMotorEncoder->GetPosition() < 5 * rotPFT) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // C3 turn
        while (abs(ahrs.GetYaw()) < 63.435) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
            robotPosL = MiddleLeftMotorEncoder->GetPosition();
            robotPosR = MiddleRightMotorEncoder->GetPosition();
        }
        // C3 -> D5
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL +(5.59 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR + (5.59 * rotPFT)) {
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
            robotPosL2 = MiddleLeftMotorEncoder->GetPosition();
            robotPosR2 = MiddleRightMotorEncoder->GetPosition();
        }
        // D5 -> A61
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL2 +(7.9 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR2 + (7.9 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // A6 turn 1
        while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() > 0) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
        }
        // A6 turn 2
        while (abs(ahrs.GetYaw()) < 21.8) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL3 = MiddleLeftMotorEncoder->GetPosition();
            robotPosR3 = MiddleRightMotorEncoder->GetPosition();
        }
        // A6 -> C11
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL3 +(13.46 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR3 + (13.46 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }

        // STOP
        driveTrain.LeftMotors->Set(0);
        driveTrain.RightMotors->Set(0);

        runIf = 0;
    } else if (runIf == 2) {
        // C1 turn
        while (abs(ahrs.GetYaw()) < 68.2) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
            robotPosL = MiddleLeftMotorEncoder->GetPosition();
            robotPosR = MiddleRightMotorEncoder->GetPosition();
        }
        // C1 -> E6
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL +(13.46 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR + (13.46 * rotPFT)) {
                driveTrain.LeftMotors->Set(.1);
                driveTrain.RightMotors->Set(.1);
        }
        // E6 turn 1
        while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
        }
        // E6 turn 2
        while (abs(ahrs.GetYaw()) < 71.565) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL2 = MiddleLeftMotorEncoder->GetPosition();
            robotPosR2 = MiddleRightMotorEncoder->GetPosition();
        }
        // E6 -> B7
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL2 +(7.9 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR2 + (7.9 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // B7 turn 1
        while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() > 0) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
        }
        // B7 turn 2
        while (abs(ahrs.GetYaw()) < 63.435) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL3 = MiddleLeftMotorEncoder->GetPosition();
            robotPosR3 = MiddleRightMotorEncoder->GetPosition();
        }
        // B7 -> C9
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL3 +(5.59 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR3 + (5.59 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // C9 turn
        while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() > 0) {
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL4 = MiddleLeftMotorEncoder->GetPosition();
            robotPosR4 = MiddleRightMotorEncoder->GetPosition();
        }
        // C9 -> C11
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL4 +(5 * rotPFT) && MiddleRightMotorEncoder->GetPosition() < robotPosR4 + (5 * rotPFT)) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }

        // STOP
        driveTrain.LeftMotors->Set(0);
        driveTrain.RightMotors->Set(0);
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
