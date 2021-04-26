#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include <frc/Servo.h>
void Robot::RobotInit() {
    m_rightTopMotor.SetInverted(true);
    m_rightMiddleMotor.SetInverted(true);
    m_rightBottomMotor.SetInverted(true);

    m_hopperRight.SetInverted(true);
    m_shooter1.SetInverted(true);
    m_timer.Reset();

    //limelight network table, sets led to off
    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    table->PutNumber("ledMode", 1);

    //servo bounds found on online so that it properly goes from closed to open
    s1.SetBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    s2.SetBounds(2.0, 1.8, 1.5, 1.2, 1.0);

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

    nt::NetworkTableInstance::GetDefault().GetTable("limelight")
        ->PutNumber("ledMode", 3);

    nt::NetworkTableInstance::GetDefault().GetTable("limelight")
        ->PutNumber("camMode", 0);
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
    LimeLight.Update();
    driveTrain.Drive();
    turret.Aim();
}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif

