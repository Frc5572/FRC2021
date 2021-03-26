#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

void Robot::RobotInit() {
    m_rightTopMotor.SetInverted(true);
    m_rightMiddleMotor.SetInverted(true);
    m_rightBottomMotor.SetInverted(true);
    m_hopperRight.SetInverted(true);
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
    // intake motor
    if (Driver.X() == true) {
        m_intake.Set(-.5);
    } else if (Driver.Y() == true) {
        m_intake.Set(.5);

    // Hopper
    if (Driver.A() == true) {
        hopper.HopperMotors->Set(.4);
    } else if (Driver.B() == true) {
        hopper.HopperMotors->Set(-.4);
    } else {
        hopper.HopperMotors->Set(0);
    }

    //Shooter Turret
    if (Driver.LB() == true) {;
        m_turret.Set(.1);
    }
    else if (Driver.RB() == true) {
        m_turret.Set(-.1);
    } else {
        m_intake.Set(0);
    }
}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif

