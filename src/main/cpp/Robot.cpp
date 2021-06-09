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
    LimeLight.Update();
}

void Robot::AutonomousPeriodic() {

}

void Robot::AutonomousInit() {
    m_timer.Reset();
    m_timer.Start();
    ahrs.Reset();
    s1.SetPosition(0);
    m_leftMiddleMotor.SetSelectedSensorPosition(0);

}

void Robot::TeleopInit() {
    s1.SetPosition(0);
}

void Robot::TeleopPeriodic() {
    driveTrain.Drive();
    turret.TurretMove();
    turret.PositionHood();
    turret.LimitCheck();
    hopper.Run();
    intake.Run();
    nonPidShooter.Run();
    // nonPidShooter.Run();
    // if (Driver.A()) {
    //     m_shooter1.Set(.6);
    //     m_shooter2.Set(.6);
    // }
    // else 
    // {
    //     m_shooter1.Set(0);
    //     m_shooter2.Set(0);
    // }
    // turret.Shoot();

    // if(Operator.POV() == 180){
    //     s1.SetPosition(0);
    // } else if(Operator.POV() == 90) {
    //     s1.SetPosition(.2);
    // } else if(Operator.POV() == 0){
    //     s1.SetPosition(1);
    // }

    // if(Operator.B()){
    //     m_shooter1.Set(.6);
    //     m_shooter2.Set(.6);
    // } else {
    //     m_shooter1.Set(0);
    //     m_shooter2.Set(0);
    // }


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
    // turret.Aim();
}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif
