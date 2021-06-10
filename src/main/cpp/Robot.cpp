#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

float limelightAngle;
float subHeight;
float distance;
float angle;
double eticks = 2048 * 9.2;
double circum = 18.84;
double wantToMove = 14 * 12; //feet to inches
double ticks = wantToMove * (eticks / circum);


void Robot::RobotInit() {
    m_leftBottomMotor.SetSelectedSensorPosition(0);
    //turret.TurretEncoder->SetPosition(0);

    driveTrain.driveInit();

    m_hopperRight.SetInverted(true);
    m_shooter1.SetInverted(true);
    m_climber2.SetInverted(true);
    m_timer.Reset();

    //limelight network table, sets led to off
    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    table->PutNumber("ledMode", 1);

    //servo bounds found on online so that it properly goes from closed to open
    s1.SetBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    compressor.Start();
    compressor.SetClosedLoopControl(true);
    climber2 = new frc::DoubleSolenoid(PCM1, 7, 0);//green
    // hopperSol = new frc::DoubleSolenoid(PCM1, 5, 2);//white
    climber1 = new frc::DoubleSolenoid(PCM2, 4, 3);//blue
    // intakeSol = new frc::DoubleSolenoid(PCM1, 6, 1);//yellow
    sol4 = new frc::DoubleSolenoid(PCM2, 5, 2);//red
    intakeSol->Set(frc::DoubleSolenoid::Value::kForward);
    hopperSol->Set(frc::DoubleSolenoid::Value::kForward);
    climber1->Set(frc::DoubleSolenoid::Value::kReverse);
    climber2->Set(frc::DoubleSolenoid::Value::kReverse);
    sol4->Set(frc::DoubleSolenoid::Value::kReverse);

    nt::NetworkTableInstance::GetDefault().GetTable("limelight")
        ->PutNumber("ledMode", 3);

    nt::NetworkTableInstance::GetDefault().GetTable("limelight")
        ->PutNumber("camMode", 0);

    firstPart = false, secondPart = false, thirdPart = false, fourthPart = false, fifthPart = false;
}

void Robot::RobotPeriodic() {
    // SetClosedLoopControl(true);
    LimeLight.Update();
}

void Robot::AutonomousInit() {
    m_timer.Reset();
    m_timer.Start();
    ahrs.Reset();
    s1.SetPosition(0);
    m_leftMiddleMotor.SetSelectedSensorPosition(0);
    LimeLight.Update();
    intakeSol->Set(frc::DoubleSolenoid::Value::kReverse);
    firstPart = false, secondPart = false, thirdPart = false, fourthPart = false, fifthPart = false;
}

void Robot::AutonomousPeriodic() {
    /*
    Servo Values
    0 = 25*
    .1 = 29*
    .2 = 33*
    .3 = 37*
    .4 = 41*
    .5 = 45*
    .6 = 49*
    .7 = 53*
    .8 = 57*
    .9 = 61*
    1 = 65*
    */

    turret.autoAim();

    std::cout << m_leftMiddleMotor.GetSelectedSensorPosition() << "\n";
    if (!firstPart) {
        if(m_timer.Get() < .25){
            m_shooter1.Set(.4);
            m_shooter2.Set(.4);
            m_turret.Set(.1);
        }
        else if (m_timer.Get() > .25 && m_timer.Get() < 3) {
            m_shooter1.Set(.4);
            m_shooter2.Set(.4);
            s1.SetPosition(.1);
        }
        else if (m_timer.Get() > 3 && m_timer.Get() < 5.5 ){
            // turret.Off();
            hopper.HopperMotors->Set(.3);
            m_shooter1.Set(.4);
            m_shooter2.Set(.4);
        }
        else if (m_timer.Get() > 5.5 && m_timer.Get() < 6) {
            hopper.HopperMotors->Set(0.0);
            m_shooter1.Set(0);
            m_shooter2.Set(0);
            s1.SetPosition(0);
        } else {
            m_leftMiddleMotor.SetSelectedSensorPosition(0);
            firstPart = true;
        }
    } else if (!secondPart) {
        m_timer.Stop();
        if (abs(m_leftMiddleMotor.GetSelectedSensorPosition()) < ticks)
        {
            driveTrain.LeftMotors->Set(.3);
            driveTrain.RightMotors->Set(.3);
            m_intake.Set(.4);
            hopper.HopperMotors->Set(.2);
            hopperSol->Set(frc::DoubleSolenoid::Value::kReverse);
        } else {
            driveTrain.LeftMotors->Set(0);
            driveTrain.RightMotors->Set(0);
            secondPart = true;
        }
    } else if (!thirdPart) {
        m_timer.Start();
        if (m_timer.Get() > 6 && m_timer.Get() < 6.5) {
            driveTrain.LeftMotors->Set(0);
            driveTrain.RightMotors->Set(0);
        } else {
            m_timer.Stop();
            thirdPart = true;
        }
    } else if (!fourthPart) {
        if (abs(m_leftMiddleMotor.GetSelectedSensorPosition()) > (12 * 9 * (eticks / circum))) {
            driveTrain.LeftMotors->Set(-.3);
            driveTrain.RightMotors->Set(-.3);
            m_shooter1.Set(.7);
            m_shooter2.Set(.7);
            s1.SetPosition(.1);
        } else {
            driveTrain.LeftMotors->Set(0);
            driveTrain.RightMotors->Set(0);
            m_timer.Start();
            fourthPart = true;
        }
    } else if (!fifthPart) {
        if (m_timer.Get() > 6.5 && m_timer.Get() < 8) {
            driveTrain.LeftMotors->Set(0);
            driveTrain.RightMotors->Set(0);
            intakeSol->Set(frc::DoubleSolenoid::Value::kForward);
            hopperSol->Set(frc::DoubleSolenoid::Value::kForward);
        } else if (m_timer.Get() > 8 && m_timer.Get() < 10) {
            intakeSol->Set(frc::DoubleSolenoid::Value::kReverse);
            hopper.HopperMotors->Set(.3);
            m_shooter1.Set(.7);
            m_shooter2.Set(.7);
        } else if (m_timer.Get() > 10 && m_timer.Get() < 16) {
            intakeSol->Set(frc::DoubleSolenoid::Value::kForward);
            hopper.HopperMotors->Set(.3);
            m_shooter1.Set(.7);
            m_shooter2.Set(.7);
        } else if (m_timer.Get() > 16 && m_timer.Get() < 16.5) {
            m_shooter1.Set(0);
            m_shooter2.Set(0);
            hopper.HopperMotors->Set(0.0);
            m_intake.Set(0);
        } else {
            fifthPart = true;
        }
    }
}

void Robot::TeleopInit() {
    s1.SetPosition(0);
}

void Robot::TeleopPeriodic() {
    driveTrain.Drive();
    turret.TurretMove();
    turret.PositionHood();
    // turret.LimitCheck();
    hopper.Run();
    nonPidShooter.Run();
    intake.Run();
}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif
