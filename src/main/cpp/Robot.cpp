#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"
#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

double motorSpeed = 0.1;
double gearRatio = 10;
double wRotationFoot = 12 * gearRatio / 18.85;
int counter = 0;
int robotPosL1 = 0;
int robotPosL2 = 0;
int robotPosL3 = 0;
int robotPosL4 = 0;
int robotPosL5 = 0;
int robotPosL6 = 0;
int robotPosL7 = 0;
int robotPosL8 = 0;
int turn = 0;

bool run_auto = true;

void Robot::RobotInit() {
    driveTrain.RightMotors->SetInverted(true);
    m_timer.Start();
    m_leftBottomMotor.RestoreFactoryDefaults();
    m_leftMiddleMotor.RestoreFactoryDefaults();
    m_leftTopMotor.RestoreFactoryDefaults();
    m_rightBottomMotor.RestoreFactoryDefaults();
    m_rightMiddleMotor.RestoreFactoryDefaults();
    m_rightTopMotor.RestoreFactoryDefaults();
    m_leftBottomMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_leftMiddleMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_leftTopMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightBottomMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightMiddleMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_rightTopMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Robot::RobotPeriodic() {
}
void Robot::AutonomousInit()    {
    shooter.InitPID();
    m_timer.Reset();
    m_timer.Start();
    ahrs.Reset();
    MiddleLeftMotorEncoder->SetPosition(0);

    //  automovement = new AutoMovement{*driveTrain.LeftMotors,
    //  *driveTrain.RightMotors,
    //  ahrs, *LeftMiddleMotorEncoder,
    //  *BottomRightMotorEncoder};
}
void Robot::AutonomousPeriodic() {
    MiddleLeftMotorEncoder->SetPosition(0);
    std::cout << ("Yaw:  \n");
    std::cout << (ahrs.GetYaw());
    // std::cout << ("\n Left Encoder \n");
    // std::cout << (MiddleLeftMotorEncoder->GetPosition());
    std::cout << ("\n Right Encoder \n");
    std::cout << (MiddleLeftMotorEncoder->GetPosition());
    if (run_auto) {
        while (MiddleLeftMotorEncoder->GetPosition() <= wRotationFoot * 5) {
            // forward 1
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }

        while (abs(ahrs.GetYaw()) < 90) {
            // turn 1
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL1 = MiddleLeftMotorEncoder->GetPosition();
        }

        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL1 + wRotationFoot * 5) {
            // forward 2
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }

        while (abs(ahrs.GetYaw()) < 135) {
            // turn 2
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL2 = MiddleLeftMotorEncoder->GetPosition();
        }

        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL2 + wRotationFoot *  sqrt(50)) {
            // forward 3
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }

        while (abs(ahrs.GetYaw()) > 90) {
            // turn 3
            driveTrain.LeftMotors->Set(-motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
            robotPosL3 = MiddleLeftMotorEncoder->GetPosition();
        }

        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL3 + wRotationFoot * 2.5) {
            // forward 4
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }

        while (abs(ahrs.GetYaw()) > 22.5) {
            // turn 4
            driveTrain.LeftMotors->Set(-motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
            robotPosL4 = MiddleLeftMotorEncoder->GetPosition();
        }

        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL4 + wRotationFoot * sqrt(31.25)) {
            // forward 5
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }

        while (abs(ahrs.GetYaw()) < 90) {
            // turn 5
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL5 = MiddleLeftMotorEncoder->GetPosition();
        }

        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL5 + wRotationFoot * 2.5) {
            // forward 6
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }

        while (abs(ahrs.GetYaw()) < 135) {
            // turn 6
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL6 = MiddleLeftMotorEncoder->GetPosition();
        }

        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL6 + wRotationFoot * sqrt(50)) {
            // forward 7
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }

        while (abs(ahrs.GetYaw()) > 90) {
            // turn 7
            driveTrain.LeftMotors->Set(-motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
            robotPosL7 = MiddleLeftMotorEncoder->GetPosition();
        }

        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL7 + wRotationFoot * 2.5) {
            // forward 8
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
        }

        while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
            // turn 8
            driveTrain.LeftMotors->Set(-motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
            robotPosL8 = MiddleLeftMotorEncoder->GetPosition();
        }

        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL8 + wRotationFoot * 5) {
            // forward 9
            autoCorrect(ahrs.GetYaw(), 0, driveTrain);

        }


        // stop motors
        driveTrain.LeftMotors->Set(0);
        driveTrain.RightMotors->Set(0);
        run_auto = false;
    }
}

int autoCorrect (int currentAngle, int correctAngle, DriveTrain dt) {
    if (currentAngle > correctAngle) {
        dt.LeftMotors->Set(0);
        dt.RightMotors->Set(motorSpeed);
        turn = 1;
        return turn;
    } else if (currentAngle < correctAngle) {
        dt.LeftMotors->Set(motorSpeed);
        dt.RightMotors->Set(0);
        turn = 2;
        return turn;
    } else {
        turn = 3;
        return turn;
    }
}

void Robot::TeleopInit() {
    shooter.InitPID();
}

void Robot::TeleopPeriodic() {
    //  LimeLight.Update();

    // shooter.RunPID();

    driveTrain.Drive();

    // shooter.Shots();

    // climber.ClimbPeriodic();

    // hopper.HopperPeriodic();
}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif
