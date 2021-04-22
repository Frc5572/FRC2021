#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"
#include "Movement/Turret.hpp"

void Robot::RobotInit() {
    m_rightTopMotor.SetInverted(true);
    m_rightMiddleMotor.SetInverted(true);
    m_rightBottomMotor.SetInverted(true);
    m_hopperRight.SetInverted(true);
    m_shooter1.SetInverted(true);
    m_timer.Reset();
    m_leftBottomMotor.SetNeutralMode(Coast);
    m_leftMiddleMotor.SetNeutralMode(Coast);
    m_leftTopMotor.SetNeutralMode(Coast);
    m_rightBottomMotor.SetNeutralMode(Coast);
    m_rightMiddleMotor.SetNeutralMode(Coast);
    m_rightTopMotor.SetNeutralMode(Coast);
    m_leftTopMotor.SetSelectedSensorPosition(0);
    m_leftMiddleMotor.SetSelectedSensorPosition(0);
    m_leftBottomMotor.SetSelectedSensorPosition(0);
    m_rightTopMotor.SetSelectedSensorPosition(0);
    m_rightMiddleMotor.SetSelectedSensorPosition(0);
    m_rightBottomMotor.SetSelectedSensorPosition(0);
    compressor.Start();
    compressor.SetClosedLoopControl(true);
    delete intake;
    delete sol2;
    delete sol1;
    delete sol3;
    delete sol4;
    intake = new frc::DoubleSolenoid(PCM1, 7, 0);//green
    sol3 = new frc::DoubleSolenoid(PCM1, 5, 2);//white
    sol2 = new frc::DoubleSolenoid(PCM2, 4, 3);//blue
    sol1 = new frc::DoubleSolenoid(PCM1, 6, 1);//yellow
    sol4 = new frc::DoubleSolenoid(PCM2, 5, 2);//red
    intake->Set(frc::DoubleSolenoid::Value::kReverse);
    sol1->Set(frc::DoubleSolenoid::Value::kReverse);
    sol3->Set(frc::DoubleSolenoid::Value::kReverse);
    sol2->Set(frc::DoubleSolenoid::Value::kReverse);
    sol4->Set(frc::DoubleSolenoid::Value::kReverse);
    delete servo;
    servo = new frc::Servo{0};
}

void Robot::RobotPeriodic() {
    // SetClosedLoopControl(true);
}

void Robot::AutonomousInit() {
    m_timer.Start();
    ahrs.Reset();
}

void Robot::AutonomousPeriodic() {

}


void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {
    driveTrain.Drive();
    turret.Aim();
    // Hopper
    // if (Driver.A() == true) {
    //     hopper.HopperMotors->Set(-.6);
    //     m_intake.Set(-.6);
    // } else if (Driver.B() == true) {
    //     hopper.HopperMotors->Set(.4);
    //     m_intake.Set(.5);
    // } else {
    //     hopper.HopperMotors->Set(0);
    //     m_intake.Set(0);
    // }

    // // Turret
    // if (Driver.LB() == true) {
    //     m_turret.Set(.1);
    // }
    // else if (Driver.RB() == true) {
    //     m_turret.Set(-.1);
    // } else {
    //     m_turret.Set(0);
    // }

    // Shooter
    // if (Driver.X() == true) {
    //     m_shooter1.Set(1);
    //     m_shooter2.Set(1);
    // } else if (Driver.Y() == true) {
    //     m_shooter1.Set(-.3);
    //     m_shooter2.Set(-.3);
    // } else {
    //     m_shooter1.Set(0);
    //     m_shooter2.Set(0);
    // }

    // // servo
    // if(Driver.A()) {
    //     servo->Set(1);
    // } else if (Driver.B()) {
    //     servo->Set(-1);
    // } else {
    //     servo->Set(0);
    // }

    if(Driver.A()) {
        intake->Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        intake->Set(frc::DoubleSolenoid::Value::kForward);
    }

    if(Driver.B()){
        sol2->Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        sol2->Set(frc::DoubleSolenoid::Value::kForward);
    }

    if(Driver.X()){
        sol1->Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        sol1->Set(frc::DoubleSolenoid::Value::kForward);
    }

    if(Driver.Y()){
        sol4->Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        sol4->Set(frc::DoubleSolenoid::Value::kForward);
    }

    if(Driver.LB()){
        sol3->Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        sol3->Set(frc::DoubleSolenoid::Value::kForward);
    }

    // if(Driver.A() == true) {
    //     intake1->Set(frc::DoubleSolenoid::Value::kForward);
    //     intake2->Set(frc::DoubleSolenoid::Value::kForward);
    //     sol1->Set(frc::DoubleSolenoid::Value::kForward);
    //     sol2->Set(frc::DoubleSolenoid::Value::kForward);
    //     sol3->Set(frc::DoubleSolenoid::Value::kForward);
    // } else if(Driver.B() == true) {
    //     intake1->Set(frc:: DoubleSolenoid::Value::kReverse);
    //     intake2->Set(frc::DoubleSolenoid::Value::kReverse);
    //     sol1->Set(frc::DoubleSolenoid::Value::kReverse);
    //     sol2->Set(frc::DoubleSolenoid::Value::kReverse);
    //     sol3->Set(frc::DoubleSolenoid::Value::kReverse);
    // } else {
    //     intake1->Set(frc::DoubleSolenoid::Value::kOff);
    //     intake2->Set(frc::DoubleSolenoid::Value::kOff);
    //     sol1->Set(frc::DoubleSolenoid::Value::kOff);
    //     sol2->Set(frc::DoubleSolenoid::Value::kOff);
    //     sol3->Set(frc::DoubleSolenoid::Value::kOff);
    // }
}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif

