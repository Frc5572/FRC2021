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

#include <frc/Servo.h>
void Robot::RobotInit() {
    // m_rightTopMotor.SetInverted(true);
    // m_rightMiddleMotor.SetInverted(true);
    // m_rightBottomMotor.SetInverted(true);
    m_timer.Reset();

    //limelight network table, sets led to off
    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    table->PutNumber("ledMode", 1);

    //servo bounds found on online so that it properly goes from closed to open
    s1.SetBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    s2.SetBounds(2.0, 1.8, 1.5, 1.2, 1.0);
}

void Robot::RobotPeriodic() {

}

void Robot::AutonomousInit() {
    m_timer.Start();
    ahrs.Reset();
}

void Robot::AutonomousPeriodic() {
    driveTrain.LeftMotors->Set(.2);
    m_leftBottomMotor.GetSelectedSensorPosition();
}


void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
    // std::cout << Driver.RT() << " " << Driver.LT() << "\n"; 
    //turret movement
    if (Driver.RT() > 0.1)
    {
        base.Set(.1);
    }
    else if (Driver.LT() > 0.1)
    {
        base.Set(-.1);
    }
    else
    {
        base.Set(0);
    }

    //servo/linear actuators for turret
    //speed is a percentage of how closed/open it is.
    if (Driver.A())
    {
        s1.SetSpeed(1);
        s2.SetSpeed(1);
    }
    else
    {
        s1.SetSpeed(-1);
        s2.SetSpeed(-1);
    }

    //shooter motors
    if (Driver.B())
    {
        m_shooter1.Set(.3);
        m_shooter2.Set(.3);
    }
    else
    {
        m_shooter1.Set(0);
        m_shooter2.Set(0);
    }

    //hopper
    if (Driver.Y())
    {
        hopper1.Set(.4);
        hopper2.Set(-.4);
    }
    else
    {
        hopper1.Set(0);
        hopper2.Set(0);
    }
    
    //intake
    if (Driver.X())
    {
        intake.Set(.1);
    }
    else
    {
        intake.Set(0);
    }

    driveTrain.Drive();
}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {return frc::StartRobot<Robot>();}
#endif
