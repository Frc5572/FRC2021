#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

int gearRatio = 10;
double rotPFT = 12* gearRatio / 18.85;

void Robot::RobotInit() {
    // m_rightTopMotor.SetInverted(true);
    // m_rightMiddleMotor.SetInverted(true);
    // m_rightBottomMotor.SetInverted(true);
    m_timer.Reset();
}

void Robot::RobotPeriodic() {
}

void Robot::AutonomousInit() {
    m_timer.Start();
    ahrs.Reset();
}

void Robot::AutonomousPeriodic() {
    driveTrain.LeftMotors->Set(.2);
    m_leftBottomMotor.GetSelectedSensorPosition();
    // while (m_leftBottomMotor.GetSelectedSensorPosition() <= 5 * rotPFT) {
    //     driveTrain.LeftMotors->Set(.2);
    //     driveTrain.RightMotors->Set(.2);
    // }
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
