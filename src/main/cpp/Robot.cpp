#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"
#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

int gearRatio = 6.9;
int robotPosL = 0;
int robotPosR = 0;
int robotPosL2 = 0;
int robotPosR2 = 0;
int robotPosL3 = 0;
int robotPosR3 = 0;
int robotPosL4 = 0;
int robotPosR4 = 0;
int robotPosL5 = 0;
int robotPosR5 = 0;
int robotPosL6 = 0;
int robotPosR6 = 0;
int robotPosL7 = 0;
int robotPosR7 = 0;
int robotPosL8 = 0;
int robotPosR8 = 0;
int robotPosL9 = 0;
int robotPosR9 = 0;
int steps = 0;
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
    std::cout << ("Yaw:  \n");
    std::cout << (ahrs.GetYaw());
    // std::cout << ("\n Left Encoder \n");
    // std::cout << (BottomLeftMotorEncoder->GetPosition());
    std::cout << ("\n Right Encoder \n");
    std::cout << (BottomRightMotorEncoder->GetPosition());
    if (run_auto) {
        while (BottomLeftMotorEncoder->GetPosition() <= 30) {
            // forward 1
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        while (abs(ahrs.GetYaw()) < 90) {
            // turn 1
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
            robotPosL2 = BottomLeftMotorEncoder->GetPosition();
            robotPosR2 = BottomRightMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL2 + 25) {
            // forward 2
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        while (abs(ahrs.GetYaw()) > 135) {
            // turn 2
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
            robotPosL3 = BottomLeftMotorEncoder->GetPosition();
            robotPosR3 = BottomRightMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL3 + 45 ) {
            // forward 3
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        while (abs(ahrs.GetYaw()) < 90) {
            // turn 3
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL4 = BottomLeftMotorEncoder->GetPosition();
            robotPosR4 = BottomRightMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL4 + 15) {
            // forward 4
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        while (abs(ahrs.GetYaw()) < 15) {
            // turn 4
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL5 = BottomLeftMotorEncoder->GetPosition();
            robotPosR5 = BottomRightMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL5 + 40) {
            // forward 5
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        while (abs(ahrs.GetYaw()) < 90) {
            // turn 5
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
            robotPosL6 = BottomLeftMotorEncoder->GetPosition();
            robotPosR6 = BottomRightMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL6 + 15) {
            // forward 6
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        while (abs(ahrs.GetYaw()) < 135) {
            // turn 6
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(-.1);
            robotPosL7 = BottomLeftMotorEncoder->GetPosition();
            robotPosR7 = BottomRightMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL7 + 45) {
            // forward 7
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        while (abs(ahrs.GetYaw()) < 90) {
            // turn 7
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL8 = BottomLeftMotorEncoder->GetPosition();
            robotPosR8 = BottomRightMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL8 + 15) {
            // forward 8
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        }
        // this probably wont work
        while (abs(ahrs.GetYaw()) <= 0 && ahrs.GetYaw()  <= 0) {
            // turn 8
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(.1);
            robotPosL9 = BottomLeftMotorEncoder->GetPosition();
            robotPosR9 = BottomRightMotorEncoder->GetPosition();
        }
        while (BottomLeftMotorEncoder->GetPosition() <= robotPosL9 + 30) {
            // forward 9
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
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
