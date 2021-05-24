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

void Robot::RobotInit() {

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
    delete sol5;
    delete sol2;
    delete intakeSol;
    delete sol3;
    delete sol4;
    sol5 = new frc::DoubleSolenoid(PCM1, 7, 0);//green
    sol3 = new frc::DoubleSolenoid(PCM1, 5, 2);//white
    sol2 = new frc::DoubleSolenoid(PCM2, 4, 3);//blue
    intakeSol = new frc::DoubleSolenoid(PCM1, 6, 1);//yellow
    sol4 = new frc::DoubleSolenoid(PCM2, 5, 2);//red
    sol5->Set(frc::DoubleSolenoid::Value::kReverse);
    intakeSol->Set(frc::DoubleSolenoid::Value::kForward);
    sol3->Set(frc::DoubleSolenoid::Value::kReverse);
    sol2->Set(frc::DoubleSolenoid::Value::kReverse);
    sol4->Set(frc::DoubleSolenoid::Value::kReverse);
    // s1.Set(0);
    // s2.Set(0);
    // delete servo;
    // servo = new frc::Servo{0};


    nt::NetworkTableInstance::GetDefault().GetTable("limelight")
        ->PutNumber("ledMode", 3);

    nt::NetworkTableInstance::GetDefault().GetTable("limelight")
        ->PutNumber("camMode", 0);
}

void Robot::RobotPeriodic() {
    // SetClosedLoopControl(true);
}

void Robot::AutonomousInit() {
    m_timer.Start();
    ahrs.Reset();
}

void Robot::AutonomousPeriodic() {
    /*
    Servo Values
    0 = 30 degrees
    .5 = 45 degrees
    1 = 60 degrees
    */
   //Getting distance from shooter thingy using limelight measurements
    limelightAngle = tan(LimeLight.ta + 40);
    subHeight = 47;
    distance = subHeight / limelightAngle;
    angle = sqrt((distance * distance) + (subHeight * subHeight));
}


void Robot::TeleopInit() {

}

void Robot::TeleopPeriodic() {

    driveTrain.Drive();
    hopper.Run();
    LimeLight.Update();

    if(Operator.POV() == 180){
        // s1.Set(1);
        // s2.Set(1);
        s1.SetPosition(0);
        // s2.SetPosition(0);
        // s2.SetSpeed(1);
    } else if(Operator.POV() == 90) {
        s1.SetPosition(.5);
        // s2.SetPosition(.5);
    } else if(Operator.POV() == 0){
        s1.SetPosition(1);
        // s2.SetPosition(1);
        // s2.SetSpeed(-1);
    }

    if(Operator.B()){
        m_shooter1.Set(.4);
        m_shooter2.Set(.4);
    } else {
        m_shooter1.Set(0);
        m_shooter2.Set(0);
    }

    // if(Driver.A()){
    //     turret.Aim();
    // }
    // turret.Aim();
    if(Operator.LB()){
        m_turret.Set(.3);
    } else if(Operator.RB()){
        m_turret.Set(-.3);
    } else {
        m_turret.Set(0);
    }
    intake.Run();
    // shooter.run();
    // if(Driver.A()){
    //     sol5->Set(frc::DoubleSolenoid::Value::kForward);
    //     sol2->Set(frc::DoubleSolenoid::Value::kForward);
    // } else {
    //     sol5->Set(frc::DoubleSolenoid::Value::kReverse);
    //     sol2->Set(frc::DoubleSolenoid::Value::kReverse);
    // }
    // if(Driver.B()){
    //     sol4->Set(frc::DoubleSolenoid::Value::kForward);
    // } else {
    //     sol4->Set(frc::DoubleSolenoid::Value::kReverse);
    // }
    // if(Driver.X()){
    //     sol3->Set(frc::DoubleSolenoid::Value::kForward);
    // } else {
    //     sol3->Set(frc::DoubleSolenoid::Value::kReverse);
    // }
    // // if(Driver.Y()){
    // //     sol2->Set(frc::DoubleSolenoid::Value::kForward);
    // // } else {
    // //     sol2->Set(frc::DoubleSolenoid::Value::kReverse);
    // // }
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


    // if(Driver.B()){
    //     m_intake.Set(.3);
    // }
    // else
    // {
    //     m_intake.Set(0);
    // }


}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif

