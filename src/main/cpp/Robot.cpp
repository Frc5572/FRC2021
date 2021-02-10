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
int diffInAngle = 0;
int robotPosL1 = 0;
int robotPosL2 = 0;
int robotPosL3 = 0;
int robotPosL4 = 0;
int robotPosL5 = 0;
int robotPosL6 = 0;
int robotPosL7 = 0;
int robotPosL8 = 0;


bool run_auto = true;

int autoCorrect(double currentAngle, double correctAngle, DriveTrain dt) {
    diffInAngle = correctAngle - currentAngle;
    if (currentAngle > 0 && correctAngle > 0) {
        if (diffInAngle > 0) {
            dt.LeftMotors->Set(0);
            dt.RightMotors->Set(motorSpeed);
        } else if (diffInAngle < 0) {
            dt.LeftMotors->Set(motorSpeed);
            dt.RightMotors->Set(0);
    } else if (currentAngle < 0 && correctAngle < 0) {
        if (diffInAngle < 0) {
            dt.LeftMotors->Set(motorSpeed);
            dt.RightMotors->Set(0);
        } else if (diffInAngle > 0) {
            dt.LeftMotors->Set(0);
            dt.RightMotors->Set(motorSpeed);
        }
    } else {
        dt.LeftMotors->Set(motorSpeed);
        dt.RightMotors->Set(motorSpeed);
    }
}
}

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
void Robot::AutonomousInit() {
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
            autoCorrect(ahrs.GetYaw(), 0, driveTrain);
        }

        while (abs(ahrs.GetYaw()) > -90) {
            // turn 1
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL1 = MiddleLeftMotorEncoder->GetPosition();
        }

        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL1 + wRotationFoot * 5) {
            // forward 2
            autoCorrect(ahrs.GetYaw(), -90, driveTrain);
        }

        while (abs(ahrs.GetYaw()) > -135) {
            // turn 2
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL2 = MiddleLeftMotorEncoder->GetPosition();
        }

        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL2 + wRotationFoot *  sqrt(50)) {
            // forward 3
            autoCorrect(ahrs.GetYaw(), -135, driveTrain);
        }

        while (abs(ahrs.GetYaw()) < -90) {
            // turn 3
            driveTrain.LeftMotors->Set(-motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
            robotPosL3 = MiddleLeftMotorEncoder->GetPosition();
        }

        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL3 + wRotationFoot * 2.5) {
            // forward 4
            autoCorrect(ahrs.GetYaw(), -90, driveTrain);
        }

        while (abs(ahrs.GetYaw()) < -22.5) {
            // turn 4
            driveTrain.LeftMotors->Set(-motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
            robotPosL4 = MiddleLeftMotorEncoder->GetPosition();
        }

        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL4 + wRotationFoot * sqrt(31.25)) {
            // forward 5
            autoCorrect(ahrs.GetYaw(), -22.5, driveTrain);
        }

        while (abs(ahrs.GetYaw()) > -90) {
            // turn 5
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL5 = MiddleLeftMotorEncoder->GetPosition();
        }

        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL5 + wRotationFoot * 2.5) {
            // forward 6
            autoCorrect(ahrs.GetYaw(), -90, driveTrain);
        }

        while (abs(ahrs.GetYaw()) > -135) {
            // turn 6
            driveTrain.LeftMotors->Set(motorSpeed);
            driveTrain.RightMotors->Set(-motorSpeed);
            robotPosL6 = MiddleLeftMotorEncoder->GetPosition();
        }

        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL6 + wRotationFoot * sqrt(50)) {
            // forward 7
            autoCorrect(ahrs.GetYaw(), -135, driveTrain);
        }

        while (abs(ahrs.GetYaw()) < -90) {
            // turn 7
            driveTrain.LeftMotors->Set(-motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
            robotPosL7 = MiddleLeftMotorEncoder->GetPosition();
        }

        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL7 + wRotationFoot * 2.5) {
            // forward 8
            autoCorrect(ahrs.GetYaw(), -90, driveTrain);
        }

        while (abs(ahrs.GetYaw()) > 0 && ahrs.GetYaw() < 0) {
            // turn 8
            driveTrain.LeftMotors->Set(-motorSpeed);
            driveTrain.RightMotors->Set(motorSpeed);
            robotPosL8 = MiddleLeftMotorEncoder->GetPosition();
        }
        ////////////////
        while (MiddleLeftMotorEncoder->GetPosition() <= robotPosL8 + wRotationFoot * 5) {
            // forward 9
            autoCorrect(ahrs.GetYaw(), 0, driveTrain);
        }
        ////////////////

        // stop motors
        driveTrain.LeftMotors->Set(0);
        driveTrain.RightMotors->Set(0);
        run_auto = false;
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
