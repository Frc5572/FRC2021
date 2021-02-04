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

    if (ahrs.GetYaw() < 180 && ahrs.GetYaw() >= 0) {
        driveTrain.LeftMotors->Set(-.3);
        driveTrain.RightMotors->Set(.3);
        //if (BottomLeftMotorEncoder->GetPosition() < 3) {
        //    driveTrain.LeftMotors->Set(.3);
        //    driveTrain.RightMotors->Set(.3);
        //}
    }
    /*
    else if (ahrs.GetYaw() > 0){
        driveTrain.LeftMotors->Set(-.3);
        driveTrain.RightMotors->Set(.3);
    }
    */
    // else if (BottomLeftMotorEncoder->GetPosition() > 6 && BottomLeftMotorEncoder->GetPosition() < 9) {
    //    driveTrain.LeftMotors->Set(.3);
    //    driveTrain.RightMotors->Set(.3);
    //}
    else {
        driveTrain.LeftMotors->Set(0);
        driveTrain.RightMotors->Set(0);
    }

    //  while (m_timer.Get() < 3) {
        // m_rightTopMotor.Set(-.1);
        // m_rightMiddleMotor.Set(-.1);
        // m_rightBottomMotor.Set(-.1);
        // m_leftTopMotor.Set(.1);
        // m_leftMiddleMotor.Set(.1);
        // m_leftBottomMotor.Set(.1);
        // driveTrain.RightMotors->Set(-.1);
        // driveTrain.LeftMotors->Set(.1);
    // }

/*
    while (m_timer.Get() < 3) {
        //  shooter.AutoPID();
        m_leftShooter.Set(.6);
        m_rightShooter.Set(.6);
        shooterHood.Set(frc::DoubleSolenoid::Value::kForward);
        //  m_hopper.Set(.3);
        continue;
    }

    while (m_timer.Get() > 3 && m_timer.Get() < 10) {
        m_hopper.Set(.3);
        m_leftShooter.Set(.6);
        m_rightShooter.Set(.6);
        shooterHood.Set(frc::DoubleSolenoid::Value::kForward);
    }
    while (m_timer.Get() > 10 && m_timer.Get() < 11) {
        shooterHood.Set(frc::DoubleSolenoid::Value::kReverse);
        m_leftShooter.Set(0);
        m_rightShooter.Set(0);
        m_hopper.Set(0.0);
        continue;
    }

    while (m_timer.Get() > 11 && m_timer.Get() < 12) {
        m_rightBottomMotor.Set(.3);
        m_rightMiddleMotor.Set(.3);

        m_leftBottomMotor.Set(-.3);
        m_leftMiddleMotor.Set(-.3);
        continue;
    }
*/




//     //dripping springs auto fixed below
//     while(m_timer.Get() < 2){
//         m_leftShooter.Set(.7);
//         m_rightShooter.Set(.7);
//     }
//     while(m_timer.Get() < 10 & m_timer.Get() > 2){
//         shooterHood.Set(frc::DoubleSolenoid::Value::kForward);
//         //intake.Set(frc::DoubleSolenoid::Value::kForward);
//         m_leftShooter.Set(.7);
//         m_rightShooter.Set(.7);
//         m_hopper.Set(.3);
//         continue;
//     }
//     while(m_timer.Get() > 10 && m_timer.Get() < 11){
//         //intake.Set(frc::DoubleSolenoid::Value::kReverse);
//         shooterHood.Set(frc::DoubleSolenoid::Value::kReverse);
//         m_leftShooter.Set(0);
//         m_rightShooter.Set(0);
//         m_hopper.Set(0.0);
//         continue;
//     }

//     while(m_timer.Get() > 11 && m_timer.Get() < 11.5){
//         m_rightBottomMotor.Set(.3);
//         m_rightMiddleMotor.Set(.3);

//         m_leftBottomMotor.Set(-.3);
//         m_leftMiddleMotor.Set(-.3);
//         continue;
//     }
//         m_rightBottomMotor.Set(0);
//         m_rightMiddleMotor.Set(0);

//         m_leftBottomMotor.Set(0);
//         m_leftMiddleMotor.Set(0);
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
