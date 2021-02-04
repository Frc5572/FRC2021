#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"
#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"






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

    //go forward 2 rotations
    if (BottomLeftMotorEncoder->GetPosition() < 2) {
        driveTrain.LeftMotors->Set(.3);
        driveTrain.RightMotors->Set(.3);
    //Turn left 90
    } else if (abs(ahrs.GetYaw()) < 90 && ahrs.GetYaw() >= 0) {
        driveTrain.LeftMotors->Set(-.3);
        driveTrain.RightMotors->Set(.3);
    //go forward 2
    } else if (BottomLeftMotorEncoder->GetPosition() > 2 && BottomLeftMotorEncoder->GetPosition() < 4) {
        driveTrain.LeftMotors->Set(.3);
        driveTrain.RightMotors->Set(.3);
    //turn left 90
    } else if (abs(ahrs.GetYaw()) < 180 && ahrs.GetYaw() >= 0) {
        driveTrain.LeftMotors->Set(-.3);
        driveTrain.RightMotors->Set(.3);
    //go forward 2
    } else if (BottomLeftMotorEncoder->GetPosition() > 4 && BottomLeftMotorEncoder->GetPosition() < 6) {
        driveTrain.LeftMotors->Set(.3);
        driveTrain.RightMotors->Set(.3);
    //turn left 90
    } else if (abs(ahrs.GetYaw()) > 90 && ahrs.GetYaw() <= 0) {
        driveTrain.LeftMotors->Set(-.3);
        driveTrain.RightMotors->Set(.3);
    //go forward 2
    } else if (BottomLeftMotorEncoder->GetPosition() > 6 && BottomLeftMotorEncoder->GetPosition() < 8) {
        driveTrain.LeftMotors->Set(.3);
        driveTrain.RightMotors->Set(.3);
    //turn left 90-
    } else if (abs(ahrs.GetYaw()) > 180 && ahrs.GetYaw() <= 0) {
        driveTrain.LeftMotors->Set(-.3);
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
