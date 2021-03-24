#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

int motorRotation = 2048;
double wheelRotation = motorRotation * 8.5;
double rotPFT = wheelRotation * 0.75;

void Robot::RobotInit() {
    m_timer.Reset();
    m_rightTopMotor.SetInverted(true);
    m_rightMiddleMotor.SetInverted(true);
    m_rightBottomMotor.SetInverted(true);
    m_leftBottomMotor.SetSelectedSensorPosition(0);
    m_leftMiddleMotor.SetSelectedSensorPosition(0);
    m_leftTopMotor.SetSelectedSensorPosition(0);
    m_rightBottomMotor.SetSelectedSensorPosition(0);
    m_rightMiddleMotor.SetSelectedSensorPosition(0);
    m_rightTopMotor.SetSelectedSensorPosition(0);
    m_leftBottomMotor.SetNeutralMode(Coast);
    m_leftMiddleMotor.SetNeutralMode(Coast);
    m_leftTopMotor.SetNeutralMode(Coast);
    m_rightBottomMotor.SetNeutralMode(Coast);
    m_rightMiddleMotor.SetNeutralMode(Coast);
    m_rightTopMotor.SetNeutralMode(Coast);
}

void Robot::RobotPeriodic() {
}

void Robot::AutonomousInit() {
    m_timer.Start();
    ahrs.Reset();
    m_leftBottomMotor.SetSelectedSensorPosition(0);
    m_leftMiddleMotor.SetSelectedSensorPosition(0);
    m_leftTopMotor.SetSelectedSensorPosition(0);
    m_rightBottomMotor.SetSelectedSensorPosition(0);
    m_rightMiddleMotor.SetSelectedSensorPosition(0);
    m_rightTopMotor.SetSelectedSensorPosition(0);
}

//Use abs(encoder) for encoder values
void Robot::AutonomousPeriodic() {
    while (abs(m_leftBottomMotor.GetSelectedSensorPosition()) < (rotPFT)) {
        driveTrain.LeftMotors->Set(.1);
        driveTrain.RightMotors->Set(.1);
    }
    driveTrain.LeftMotors->Set(0);
    driveTrain.RightMotors->Set(0);
}

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
    driveTrain.Drive();
}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif
