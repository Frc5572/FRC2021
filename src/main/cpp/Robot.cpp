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
// double tpft = 10500;
double eticks = 2048 * 6.66666;
double circum = 13.7;
double wantToMove = 14 * 12; //feet to inches
double ticks = wantToMove * ((double)eticks / circum);



void Robot::RobotInit() {
    m_leftBottomMotor.SetSelectedSensorPosition(0);

    driveTrain.driveInit();

    m_hopperRight.SetInverted(true);
    m_shooter1.SetInverted(true);
    m_timer.Reset();

    //limelight network table, sets led to off
    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    table->PutNumber("ledMode", 1);

    //servo bounds found on online so that it properly goes from closed to open
    s1.SetBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    // s2.SetBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    compressor.Start();
    compressor.SetClosedLoopControl(true);
    delete climber2;
    delete climber1;
    delete intakeSol;
    delete hopperSol;
    delete sol4;
    climber2 = new frc::DoubleSolenoid(PCM1, 7, 0);//green
    hopperSol = new frc::DoubleSolenoid(PCM1, 5, 2);//white
    climber1 = new frc::DoubleSolenoid(PCM2, 4, 3);//blue
    intakeSol = new frc::DoubleSolenoid(PCM1, 6, 1);//yellow
    sol4 = new frc::DoubleSolenoid(PCM2, 5, 2);//red
    hopperSol->Set(frc::DoubleSolenoid::Value::kReverse);
    intakeSol->Set(frc::DoubleSolenoid::Value::kForward);
    hopperSol->Set(frc::DoubleSolenoid::Value::kForward);
    climber1->Set(frc::DoubleSolenoid::Value::kReverse);
    sol4->Set(frc::DoubleSolenoid::Value::kReverse);
    // s1.Set(0);
    // s2.Set(0);
    // delete servo;
    // servo = new frc::Servo{0};


    nt::NetworkTableInstance::GetDefault().GetTable("limelight")
        ->PutNumber("ledMode", 3);

    nt::NetworkTableInstance::GetDefault().GetTable("limelight")
        ->PutNumber("camMode", 0);

    firstPart = false, secondPart = false, thirdPart = false, fourthPart = false, fifthPart = false, sixthPart = false;
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
    //Getting distance from shooter thingy using limelight measurements
    // limelightAngle = tan(LimeLight.ta + 40);
    // subHeight = 47;
    // distance = subHeight / limelightAngle;
    // angle = sqrt((distance * distance) + (subHeight * subHeight));
    // if(m_timer.Get() < 5){
    //     turret.Aim();
    // }
    // turret.Aim();

    turret.autoAim();
            std::cout << m_leftMiddleMotor.GetSelectedSensorPosition() << "\n";
    if (!firstPart) {
        if (m_timer.Get() < 5) {
            m_shooter1.Set(.4);
            m_shooter2.Set(.4);
            s1.SetPosition(.2);
        }
        else if (m_timer.Get() > 5 && m_timer.Get() < 13 ){
            // turret.Off();
            hopper.HopperMotors->Set(.3);
            m_shooter1.Set(.4);
            m_shooter2.Set(.4);
        }
        else if (m_timer.Get() > 13 && m_timer.Get() < 14) {
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
            driveTrain.LeftMotors->Set(-.1);
            driveTrain.RightMotors->Set(-.1);
        } else {
            driveTrain.LeftMotors->Set(0);
            driveTrain.RightMotors->Set(0);
            secondPart = true;
        }
    } else if (!thirdPart) {
        m_timer.Start();
        if (m_timer.Get() >14 && m_timer.Get() < 16) {
            driveTrain.LeftMotors->Set(0);
            driveTrain.RightMotors->Set(0);
        } else {
            m_timer.Stop();
            thirdPart = true;
        }
    } else if (!fourthPart) {
        if (abs(m_leftMiddleMotor.GetSelectedSensorPosition()) > (12 * 6 * (eticks / circum))) {
            driveTrain.LeftMotors->Set(.1);
            driveTrain.RightMotors->Set(.1);
        } else {
            driveTrain.LeftMotors->Set(0);
            driveTrain.RightMotors->Set(0);
            m_timer.Start();
            fourthPart = true;
        }
    } else if (!fifthPart) {
        if (m_timer.Get() >16 && m_timer.Get() < 18) {
            driveTrain.LeftMotors->Set(0);
            driveTrain.RightMotors->Set(0);
        } else if (m_timer.Get() > 18 && m_timer.Get() < 23) {
            m_shooter1.Set(.4);
            m_shooter2.Set(.4);
        } else if (m_timer.Get() > 23 && m_timer.Get() < 31 ) {
            hopper.HopperMotors->Set(.3);
            m_shooter1.Set(.4);
            m_shooter2.Set(.4);
        } else if (m_timer.Get() > 31 && m_timer.Get() < 32) {
            m_shooter1.Set(0);
            m_shooter2.Set(0);
            hopper.HopperMotors->Set(0.0);
        } else {
            fifthPart = true;
        }

    }


            std::cout << m_leftMiddleMotor.GetSelectedSensorPosition() << "\n";
    // turret.autoAim();
    // m_leftMiddleMotor.SetSelectedSensorPosition(0);
    // m_leftMiddleMotor.GetSensorCollection();

    // // m_timer.Reset();

}

void Robot::TeleopInit() {
    // LimeLight.Update();
    s1.SetPosition(0);
}

void Robot::TeleopPeriodic() {
    turret.Aim();
    // LimeLight.Update();

    driveTrain.Drive();
    hopper.Run();
    // turret.Aim();

    if(Operator.POV() == 180){
        s1.SetPosition(0);
    } else if(Operator.POV() == 90) {
        s1.SetPosition(.2);
    } else if(Operator.POV() == 0){
        s1.SetPosition(1);
    }

    if(Operator.B()){
        m_shooter1.Set(.6);
        m_shooter2.Set(.6);
    } else {
        m_shooter1.Set(0);
        m_shooter2.Set(0);
    }


    // if(Operator.LB()){
    //     m_turret.Set(-.2);
    // } else if(Operator.RB()){
    //     m_turret.Set(.2);
    // } else {
    //     m_turret.Set(0);
    // }

    // if(Driver.A()){
    //     turret.Aim();
    // }
    // turret.Aim();
    // if(Operator.LB()){
    //     m_turret.Set(.3);
    // } else if(Operator.RB()){
    //     m_turret.Set(-.3);
    // } else {
    //     m_turret.Set(0);
    // }
    // intake.Run();
    // shooter.run();

    // if(Driver.A()){
    //     climber2->Set(frc::DoubleSolenoid::Value::kForward);
    //     climber1->Set(frc::DoubleSolenoid::Value::kForward);
    // } else {
    //     climber2->Set(frc::DoubleSolenoid::Value::kReverse);
    //     climber2->Set(frc::DoubleSolenoid::Value::kReverse);
    // }
    // if(Driver.B()){
    //     sol4->Set(frc::DoubleSolenoid::Value::kForward);
    // } else {
    //     sol4->Set(frc::DoubleSolenoid::Value::kReverse);
    // }
    // if(Driver.X()){
    //     hopperSol->Set(frc::DoubleSolenoid::Value::kForward);
    // } else {
    //     hopperSol->Set(frc::DoubleSolenoid::Value::kReverse);
    // }
    // if(Driver.LB()){
    //     intakeSol->Set(frc::DoubleSolenoid::Value::kReverse);
    // } else {
    //     intakeSol->Set(frc::DoubleSolenoid::Value::kForward);
    // }
    // Shooter shooter.Test();
    // turret.Aim();
    // if (Driver.A())
    // {
    //     // shooter.run();
    //     m_shooter1.Set(.7);
    //     m_shooter2.Set(.7);
    // }
    // else
    // {
    //     m_shooter1.Set(0);
    //     m_shooter2.Set(0);
    // }
}

void Robot::TestInit() {
    // LimeLight.Update();
}

void Robot::TestPeriodic() {
    turret.Aim();
}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif
