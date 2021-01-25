#ifndef SHOOTER_HPP
#define SHOOTER_HPP

#include <frc/DoubleSolenoid.h>
#include "rev/CANSparkMax.h"
#include "AHRS.h"
#include "Movement/ControllerManager.hpp"
#include <frc/SpeedControllerGroup.h>
#include <frc/SmartDashboard/SmartDashboard.h>

class Shooter{
    public:

    Shooter(
        rev::CANSparkMax &LeftMotor,
        rev::CANSparkMax &RightMotor,
        frc::DoubleSolenoid &Hood,
        FRC5572Controller &Operator
    );

    void Shot();
    void Calucate();
    void TestRPM();
    void Test();
    void AutoPID();
    void RunPID();
    void InitPID();


    void Shots();

    // void BumberShot();
    // void InitShot();
    // void TrenchShot();
    int AutoSelection = 0;
    bool Tracked = true, hood = false;
    double Power = 0, Distance = 0, rpm = 0, leftRPM = 0, rightRPM = 0, SP = 0;

    // // PID coefficients
    // double kP = 0.000000000000001, kI = .2, kD = 1, kIz = .2, kFF = 0, kMaxOutput = 1, kMinOutput = -1, SetP = 700;
    double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0 ,SetP = 700;

    /* Notes of power percentage and RPMs to make shot at certain distances*/
    // Bumber shot = %- RPM - 
    // Initiation line = %- RPM - 
    // Trench = %- RPM - 
    // Color Wheel = %- RMP - 

    rev::CANPIDController *m_pidController ;
    rev::CANPIDController *m_pidController2 ;

    frc::SpeedControllerGroup* shooterMotors;
    
    FRC5572Controller* Operator;

    rev::CANSparkMax* leftMotor;
    rev::CANSparkMax* rightMotor;

    frc::DoubleSolenoid* Hood;

    rev::CANEncoder* leftMotorEncoder    ;
    rev::CANEncoder* rightMotorEncoder   ;
};

#endif