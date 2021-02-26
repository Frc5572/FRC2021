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
int robotPosL1 = 0;
int robotPosL2 = 0;
int robotPosL3 = 0;
int robotPosL4 = 0;
int robotPosL5 = 0;
int robotPosL6 = 0;
int robotPosL7 = 0;
int robotPosL8 = 0;

std::tuple <double, double> E1;
std::tuple <double, double> D2;
std::tuple <double, double> B2;
std::tuple <double, double> B4;
std::tuple <double, double> D6;
std::tuple <double, double> D7;
std::tuple <double, double> B8;
std::tuple <double, double> B9;
std::tuple <double, double> D11;
std::tuple <double, double> B11;
//                              x, y
std::tuple E1 = std::make_tuple(0, 0);
std::tuple D2 = std::make_tuple(2.5, 2.5);
std::tuple B2 = std::make_tuple(7.5, 2.5);
std::tuple B4 = std::make_tuple(7.5, 7.5);
std::tuple D6 = std::make_tuple(2.5, 12.5);
std::tuple D7 = std::make_tuple(2.5, 15);
std::tuple B8 = std::make_tuple(7.5, 17.5);
std::tuple B9 = std::make_tuple(7.5, 20);
std::tuple D11 = std::make_tuple(2.5, 25);
std::tuple B11 = std::make_tuple(7.5, 25);

// double gridReturnAngle(double startPointX, double startPointY, double endPointX, double endPointY, float gyroPos ){
//     // doing the math here
//     double angle = atan2(endPointY - startPointY, endPointX - startPointX) * 180 / PI;
//     double deltaY = (endPointY - startPointY);
//     double deltaX = (endPointX - startPointX);
//     double result = mathToDegrees(atan2(deltaY, deltaX));
// }

// double mathToDegrees(double y, double x){
//     double radians = atan(y/x);
//     double degrees = radians * (180.0/3.141592653589793238463);
//     return degrees;
// }

double gridReturnAngle(double startPointX, double startPointY, double endPointX, double endPointY, double gyroPos){
    double x, y, result = 0;
    // doing the math here
    double rise = endPointY - startPointY;
    double run = endPointX - startPointX;
    result = atan(rise/run);
}

int gridReturnDistance(double startPointX, double startPointY, double endPointX, double endPointY) {
    int calculatedDistance = 0.0;
    // doing the math here
    return wRotationFoot * sqrt(pow(endPointX - startPointX, 2) + pow(endPointY - startPointY, 2));
}


void Robot::RobotInit() {
    m_timer.Start();
    frc::SmartDashboard::PutString("Path", "Path 1");
}

void Robot::RobotPeriodic() {
}

void Robot::AutonomousInit() {
    shooter.InitPID();
    m_timer.Reset();
    m_timer.Start();

    //  automovement = new AutoMovement{*driveTrain.LeftMotors,
    //  *driveTrain.RightMotors,
    //  ahrs, *BottomLeftMotorEncoder,
    //  *BottomRightMotorEncoder};
}
void Robot::AutonomousPeriodic() {
    if (runAuto) {
        auto pathName = frc::SmartDashboard::GetString("Path", "Path B");
        if (pathName == "Path A") {
        } else if (pathName == "Path B") {
            // while(MiddleLeftMotorEncoder->GetPosition() < gridReturnDistance(get<0>(D2), get<1>(D2), get<0>(B2), get<1>(B2))) {
            //     //move motors forward
            // }
            while (BottomLeftMotorEncoder->GetPosition() <=  gridReturnDistance(std::get<0>(D2), std::get<1>(D2), std::get<0>(B2), std::get<1>(B2))) {
                // forward 1
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (ahrs.GetYaw() < gridReturnAngle(std::get<0>(D2), std::get<1>(D2), std::get<0>(B2), std::get<1>(B2))) {
                // turn 1
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(-motorSpeed);
                robotPosL1 = BottomLeftMotorEncoder->GetPosition();
            }
            // calculate
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL1 + wRotationFoot * 5) {
                // forward 2
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) < 135) {
                // turn 2
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(-motorSpeed);
                robotPosL2 = BottomLeftMotorEncoder->GetPosition();
            }
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL2 + wRotationFoot *  sqrt(50)) {
                // forward 3
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) > 90) {
                // turn 3
                driveTrain.LeftMotors->Set(-motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
                robotPosL3 = BottomLeftMotorEncoder->GetPosition();
            }
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL3 + wRotationFoot * 2.5) {
                // forward 4
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) > 22.5) {
                // turn 4
                driveTrain.LeftMotors->Set(-motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
                robotPosL4 = BottomLeftMotorEncoder->GetPosition();
            }
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL4 + wRotationFoot * sqrt(31.25)) {
                // forward 5
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) < 90) {
                // turn 5
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(-motorSpeed);
                robotPosL5 = BottomLeftMotorEncoder->GetPosition();
            }
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL5 + wRotationFoot * 2.5) {
                // forward 6
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) < 135) {
                // turn 6
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(-motorSpeed);
                robotPosL6 = BottomLeftMotorEncoder->GetPosition();
            }
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL6 + wRotationFoot * sqrt(50)) {
                // forward 7
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            while (abs(ahrs.GetYaw()) > 90) {
                // turn 7
                driveTrain.LeftMotors->Set(-motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
                robotPosL7 = BottomLeftMotorEncoder->GetPosition();
            }
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL7 + wRotationFoot * 2.5) {
                // forward 8
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }
            // this probably wont work
            while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
                // turn 8
                driveTrain.LeftMotors->Set(-motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
                robotPosL8 = BottomLeftMotorEncoder->GetPosition();
            }
            while (BottomLeftMotorEncoder->GetPosition() <= robotPosL8 + wRotationFoot * 5) {
                // forward 9
                driveTrain.LeftMotors->Set(motorSpeed);
                driveTrain.RightMotors->Set(motorSpeed);
            }

            // stop motors
            driveTrain.LeftMotors->Set(0);
            driveTrain.RightMotors->Set(0);
            runAuto = false;
        } else if (pathName == "Path C") {
        }
    runAuto == false;
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
