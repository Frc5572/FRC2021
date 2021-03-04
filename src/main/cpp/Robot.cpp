#include <iostream>
#include <string>
#include <tuple>
#include <math.h>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"
#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"
bool runAuto = true;
bool gridReturnValue = true;
double motorSpeed = 0.1;
double gearRatio = 10;
double wRotationFoot = 12 * gearRatio / 18.85;
bool blueRed = false;
int robotPosL1 = 0;
int robotPosL2 = 0;
int robotPosL3 = 0;
int robotPosL4 = 0;
int robotPosL5 = 0;
int robotPosL6 = 0;
int robotPosL7 = 0;
int robotPosL8 = 0;

void Robot::RobotInit() {
    m_timer.Start();
    frc::SmartDashboard::PutString("Path", "Path B");
    frc::SmartDashboard::PutString("Path Color", "Path Blue");
}

void Robot::RobotPeriodic() {
}

void Robot::AutonomousInit() {
    shooter.InitPID();
    m_timer.Reset();
    m_timer.Start();
}
void Robot::AutonomousPeriodic() {
    if (runAuto) {
        auto pathName = frc::SmartDashboard::GetString("Path", "Path B");
        auto pathColor = frc::SmartDashboard::GetString("Path", "Path Blue");
        if (blueRed == false) {
            while (abs(ahrs.GetYaw()) < 26.57) {
                // turn 1
                driveTrain.LeftMotors->Set(-motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
                robotPosL1 = BottomLeftMotorEncoder->GetPosition();
            }
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL1 + wRotationFoot * 5.6) {
                // forward 1
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) < 45 + 26.57) {
                // turn 2
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(-motorSpeed);
                robotPosL2 = BottomLeftMotorEncoder->GetPosition();
            }
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL2 + wRotationFoot * sqrt(50)) {
                // forward 2
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) > 45) {
                // turn 3
                driveTrain.LeftMotors->Set(-motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
                robotPosL3 = BottomLeftMotorEncoder->GetPosition();
            }
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL3 + wRotationFoot *  sqrt(50)) {
                // forward 3
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) < 90) {
                // turn 4
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(-motorSpeed);
                robotPosL4 = BottomLeftMotorEncoder->GetPosition();
            }
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL4 + wRotationFoot *  5) {
                // forward 3
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }

            // stop motors
            driveTrain.LeftMotors->Set(0);
            driveTrain.RightMotors->Set(0);
            runAuto = false;
        } else if (blueRed == true) {
            while (abs(ahrs.GetYaw()) < 68.2) {
                // turn 1
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(-motorSpeed);
                robotPosL1 = BottomLeftMotorEncoder->GetPosition();
            }
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL1 + wRotationFoot *  sqrt(181.25)) {
                // forward 2
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) > 45) {
                // turn 2
                driveTrain.LeftMotors->Set(-motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
                robotPosL2 = BottomLeftMotorEncoder->GetPosition();
            }
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL2 + wRotationFoot * sqrt(50)) {
                // forward 3
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) < 135) {
                // turn 3
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(-motorSpeed);
                robotPosL3 = BottomLeftMotorEncoder->GetPosition();
            }
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL3 + wRotationFoot * sqrt(50)) {
                // forward 4
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) > 90) {
                // turn 4
                driveTrain.LeftMotors->Set(-motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
                robotPosL4 = BottomLeftMotorEncoder->GetPosition();
            }
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL4 + wRotationFoot * 2.5) {
                // forward 5
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            // stop motors
            driveTrain.LeftMotors->Set(0);
            driveTrain.RightMotors->Set(0);
            runAuto = false;
        }
    }
}
void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif
