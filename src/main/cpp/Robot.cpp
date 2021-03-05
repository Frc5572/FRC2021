#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

void Robot::RobotInit() {
    driveTrain.RightMotors->SetInverted(true);
    m_timer.Start();
}

void Robot::RobotPeriodic() {
}

void Robot::AutonomousInit() {
    m_timer.Reset();
    m_timer.Start();
    ahrs.Reset();

}

void Robot::AutonomousPeriodic() {
    m_leftBottomMotor.Set(.3);
    m_rightBottomMotor.Set(.3);
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
