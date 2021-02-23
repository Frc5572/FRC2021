#include <iostream>
#include <string>
#include <tuple>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"
#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"
bool runAuto = true;
bool gridReturnValue = true;
// std::tuple <double, double> E1;
// std::tuple <double, double> D2;
// std::tuple <double, double> B2;
// std::tuple <double, double> B4;
// std::tuple <double, double> D6;
// std::tuple <double, double> D7;
// std::tuple <double, double> B8;
// std::tuple <double, double> B9;
// std::tuple <double, double> D11;
// std::tuple <double, double> B11;
// //                              x, y
// std::tuple E1 = std::make_tuple(0, 0);
// std::tuple D2 = std::make_tuple(2.5, 2.5);
// std::tuple B2 = std::make_tuple(7.5, 2.5);
// std::tuple B4 = std::make_tuple(7.5, 7.5);
// std::tuple D6 = std::make_tuple(2.5, 12.5);
// std::tuple D7 = std::make_tuple(2.5, 15);
// std::tuple B8 = std::make_tuple(7.5, 17.5);
// std::tuple B9 = std::make_tuple(7.5, 20);
// std::tuple D11 = std::make_tuple(2.5, 25);
// std::tuple B11 = std::make_tuple(7.5, 25);

double gridReturnAngle(double startPointX, double startPointY, double endPointX, double endPointY) {
    int calculatedAngle;
    // doing the math here
    /*

    */
    return calculatedAngle;
}

int gridReturnDistance(/*std::tuple startPoint, std::tuple endpoint*/) {
    int calculatedDistance;
    // doing the math here
    /*

    */
    return calculatedDistance;
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
    // if (runAuto){
    //     while(MiddleLeftMotorEncoder->GetPosition() < gridReturnDistance(get<0>(D2), get<1>(D2), get<0>(B2), get<1>(B2))) {
    //         //move motors forward
    //     }
    // }
    auto pathName = frc::SmartDashboard::GetString("Path", "Path 1");
    if (pathName == "Path 1")
    {

    }
    else if (pathName == "Path 2")
    {

    }
    else if (pathName == "Path 3")
    {

    }
    runAuto == false;
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
