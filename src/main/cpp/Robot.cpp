#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

void Robot::RobotInit() {
    // m_rightTopMotor.SetInverted(true);
    // m_rightMiddleMotor.SetInverted(true);
    // m_rightBottomMotor.SetInverted(true);
    m_timer.Reset();
}

void Robot::RobotPeriodic() {
}

void Robot::AutonomousInit() {
    leftMiddleMotor.SetSelectedSensorPosition(0);
    m_timer.Start();
    ahrs.Reset();
}

void Robot::AutonomousPeriodic() {
    if (leftMiddleMotor.GetSelectedSensorPosition() < TANK_ONE_ROTATION)
    {
        left.Set(.1);
        right.Set(.1);
    }
    else 
    {
        left.Set(0);
        right.Set(0);
    }
}


void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
    // driveTrain.Drive();

}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif
