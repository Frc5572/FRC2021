#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"
#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"






void Robot::RobotInit() {
    m_timer.Start();
    compressor.Start();
}

void Robot::RobotPeriodic() {
}
void Robot::AutonomousInit()     {
    // shooter.InitPID();
    m_timer.Reset();
    m_timer.Start();

    //  automovement = new AutoMovement{*driveTrain.LeftMotors,
    //  *driveTrain.RightMotors,
    //  ahrs, *BottomLeftMotorEncoder,
    //  *BottomRightMotorEncoder};
}
void Robot::AutonomousPeriodic() {
    //  while (m_timer.Get() < 15)
    //  {
    //  automovement->TestDrive();
    //  continue;
    //  }


    while (m_timer.Get() < 3) {
        //  shooter.AutoPID();
        m_leftShooter.Set(.6);
        m_rightShooter.Set(.6);
        // shooterHood.Set(frc::DoubleSolenoid::Value::kForward);
        //  m_hopper.Set(.3);
        continue;
    }

    while (m_timer.Get() > 3 && m_timer.Get() < 10) {
        m_hopper.Set(.3);
        m_leftShooter.Set(.6);
        m_rightShooter.Set(.6);
        // shooterHood.Set(frc::DoubleSolenoid::Value::kForward);
    }
    while (m_timer.Get() > 10 && m_timer.Get() < 11) {
        // shooterHood.Set(frc::DoubleSolenoid::Value::kReverse);
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
    m_rightBottomMotor.Set(0);
    m_rightMiddleMotor.Set(0);

    m_leftBottomMotor.Set(0);
    m_leftMiddleMotor.Set(0);




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
    /// shooter.InitPID();
    frc::DoubleSolenoid exampleDouble{1, 2};
    frc::DoubleSolenoid anotherDoubleSolenoid{ 9, 1, 2};
    exampleDouble.Set(frc::DoubleSolenoid::Value::kOff);
    exampleDouble.Set(frc::DoubleSolenoid::Value::kForward);
    exampleDouble.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Robot::TeleopPeriodic() {
     LimeLight.Update();

    // shooter.RunPID();

    driveTrain.Drive();

    // shooter.Shots();

    // climber.ClimbPeriodic();

    hopper.HopperPeriodic();

    if (Driver.A() == true) {
        intake1.Set(frc::DoubleSolenoid::Value::kForward);
        intake2.Set(frc::DoubleSolenoid::Value::kForward);

    } else if (Driver.B() == true) {
        intake1.Set(frc::DoubleSolenoid::Value::kReverse);
        intake2.Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        intake1.Set(frc::DoubleSolenoid::Value::kOff);
        intake2.Set(frc::DoubleSolenoid::Value::kOff);
    }
}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif
