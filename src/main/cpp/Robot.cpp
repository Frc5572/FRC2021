#include <iostream>
#include <string>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"
#include "cameraserver/CameraServer.h"

#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"






void Robot::RobotInit() {
    m_timer.Start();
    // compressor.Start();
    compressor.SetClosedLoopControl(true);
    bool enabled = compressor.Enabled();
    bool pressureSwitch = compressor.GetPressureSwitchValue();
    double current = compressor.GetCompressorCurrent();  
    frc::SmartDashboard::PutNumber("id1", 0);
    frc::SmartDashboard::PutNumber("id2", 7);
    frc::SmartDashboard::PutNumber("id3", 3);
    frc::SmartDashboard::PutNumber("id4", 6);
}

void Robot::RobotPeriodic() {
}

void Robot::AutonomousInit()     {
    // shooter.InitPID();
    m_timer.Reset();
    m_timer.Start();

    //  automovement = new AutoMovement{*driveTrain.LeftMotors,
    //  *driveTrain.RightMotors,
    //  ahrs, *BottomLeftMotorEncoder,
    //  *BottomRightMotorEncoder};
}
void Robot::AutonomousPeriodic() {

}

void Robot::TeleopInit() {
    /// shooter.InitPID();
    /* intake1.Set(frc::DoubleSolenoid::Value::kOff);
    intake2.Set(frc::DoubleSolenoid::Value::kOff);
    Solenoid.Set(frc::DoubleSolenoid::Value::kOff;
    Solenoid2.Set(frc::DoubleSolenoid::Value::kOff);
    Solenoid3.Set(frc::DoubleSolenoid::Value::kOff); */
    delete intake1;
    delete intake2;
    intake1 = new frc::DoubleSolenoid(PCM1, frc::SmartDashboard::GetNumber("id1", 0), frc::SmartDashboard::GetNumber("id2", 7));
    intake2 = new frc::DoubleSolenoid(PCM1, frc::SmartDashboard::GetNumber("id2", 3), frc::SmartDashboard::GetNumber("id3", 6));
    intake1->Set(frc::DoubleSolenoid::Value::kOff);
    intake2->Set(frc::DoubleSolenoid::Value::kOff);
    
    m_timer.Reset();
    m_timer.Start();
}

void Robot::TeleopPeriodic() {
     LimeLight.Update();

    // shooter.RunPID();

    driveTrain.Drive();

    // shooter.Shots();

    // climber.ClimbPeriodic();

   //hopper.HopperPeriodic();

    // if (Driver.A() == true) {
    //     intake1.Set(frc::DoubleSolenoid::Value::kForward);
    //     intake2.Set(frc::DoubleSolenoid::Value::kForward);

    // } else if (Driver.B() == true) {
    //     intake1.Set(frc::DoubleSolenoid::Value::kReverse);
    //     intake2.Set(frc::DoubleSolenoid::Value::kReverse);
    // } else {
    //     intake1.Set(frc::DoubleSolenoid::Value::kOff);
    //     intake2.Set(frc::DoubleSolenoid::Value::kOff);
    // } 

    // if (Driver.A()) {
    //     intake1->Set(frc::DoubleSolenoid::Value::kForward);
    //     intake2->Set(frc::DoubleSolenoid::Value::kForward);
    //     std::cout << "a" << "\n";
    //     std::cout << intake1->Get() << "\n"  <<
    //     intake1->GetError().GetMessage() << "\n";
    // } else if (Driver.B()) {
    //     intake1->Set(frc::DoubleSolenoid::Value::kReverse);
    //     intake2->Set(frc::DoubleSolenoid::Value::kReverse);
    // }

    if (Driver.A())
    {
        intake1->Toggle()
    }

    /* while(m_timer.Get() > 5){
        intake1.Set(frc::DoubleSolenoid::Value::kForward);
        intake2.Set(frc::DoubleSolenoid::Value::kForward);
    } */
}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif
