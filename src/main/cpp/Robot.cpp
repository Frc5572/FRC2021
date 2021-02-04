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

    if (abs(ahrs.GetYaw()) <= 90) {
        //turn 1
        driveTrain.LeftMotors->Set(-.3);
        driveTrain.RightMotors->Set(.3);
        robotPosL = BottomLeftMotorEncoder->GetPosition();
        robotPosR = BottomRightMotorEncoder->GetPosition();
    } else if (BottomLeftMotorEncoder->GetPosition() <= robotPosL + (3 * gearRatio) && BottomRightMotorEncoder->GetPosition() < robotPosR + (3 * gearRatio)) {
        //forward 1
        driveTrain.LeftMotors->Set(.3);
        driveTrain.RightMotors->Set(.3);
    } else if (abs(ahrs.GetYaw()) <= 180) {
        //turn 2
        driveTrain.LeftMotors->Set(-.3);
        driveTrain.RightMotors->Set(.3);
        robotPosL2 = BottomLeftMotorEncoder->GetPosition();
        robotPosR2 = BottomRightMotorEncoder->GetPosition();
    } else if (BottomLeftMotorEncoder->GetPosition() <= robotPosL2 + (3 * gearRatio) && BottomRightMotorEncoder->GetPosition() < robotPosR2 + (3 * gearRatio)) {
        //forward 2
        driveTrain.LeftMotors->Set(.3);
        driveTrain.RightMotors->Set(.3);
    } else if (abs(ahrs.GetYaw()) >= 90) {
        //turn 3
        driveTrain.LeftMotors->Set(-.3);
        driveTrain.RightMotors->Set(.3);
        robotPosL3 = BottomLeftMotorEncoder->GetPosition();
        robotPosR3 = BottomRightMotorEncoder->GetPosition();
    } else if (BottomLeftMotorEncoder->GetPosition() <= robotPosL3 + (3 * gearRatio) && BottomRightMotorEncoder->GetPosition() < robotPosR3 + (3 * gearRatio)) {
        //forward 3
        driveTrain.LeftMotors->Set(.3);
        driveTrain.RightMotors->Set(.3);
    } else if (abs(ahrs.GetYaw()) >= 0) {
        //turn 4
        driveTrain.LeftMotors->Set(-.3);
        driveTrain.RightMotors->Set(.3);
        robotPosL4 = BottomLeftMotorEncoder->GetPosition();
        robotPosR4 = BottomRightMotorEncoder->GetPosition();
    } else if (BottomLeftMotorEncoder->GetPosition() <= robotPosL4 + (3 * gearRatio) && BottomRightMotorEncoder->GetPosition() < robotPosR4 + (3 * gearRatio)) {
        //forward 4
        driveTrain.LeftMotors->Set(.3);
        driveTrain.RightMotors->Set(.3);
    } else {
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
