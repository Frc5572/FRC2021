#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"
#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

int gearRatio = 6.9;
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
    if (runIf){
        while (BottomLeftMotorEncoder->GetPosition() <= 6 * gearRatio && BottomRightMotorEncoder->GetPosition() < 6 * gearRatio) {
            //forward 1
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);

        }
        while (abs(ahrs.GetYaw()) < 117.565) {
            //turn 1
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
            robotPosL = BottomLeftMotorEncoder->GetPosition();
            robotPosR = BottomRightMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL +(6.71 * gearRatio) && BottomRightMotorEncoder->GetPosition() < robotPosR + (6.71 * gearRatio)) {
            //forward 1
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL2 = BottomLeftMotorEncoder->GetPosition();
            robotPosR2 = BottomRightMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL2 +(6 * gearRatio) && BottomRightMotorEncoder->GetPosition() < robotPosR2 + (6 * gearRatio)) {
            //forward 1
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        while (abs(ahrs.GetYaw()) < 135) {
            //turn 1
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL3 = BottomLeftMotorEncoder->GetPosition();
            robotPosR3 = BottomRightMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL3 +(4.24 * gearRatio) && BottomRightMotorEncoder->GetPosition() < robotPosR3 + (4.24 * gearRatio)) {
            //forward 1
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL4 = BottomLeftMotorEncoder->GetPosition();
            robotPosR4 = BottomRightMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL4 +(4.24 * gearRatio) && BottomRightMotorEncoder->GetPosition() < robotPosR4 + (4.24 * gearRatio)) {
            //forward 1
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        while (abs(ahrs.GetYaw()) >= 0 ) {
            //turn 1
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
            robotPosL5 = BottomLeftMotorEncoder->GetPosition();
            robotPosR5 = BottomRightMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL5 +(12 * gearRatio) && BottomRightMotorEncoder->GetPosition() < robotPosR5 + (12 * gearRatio)) {
            //forward 1
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);

        }
        while (abs(ahrs.GetYaw()) >= 0 ) {
            //turn 1
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
            robotPosL6 = BottomLeftMotorEncoder->GetPosition();
            robotPosR6 = BottomRightMotorEncoder->GetPosition();
        }

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
