#include "Robot.h"
#include "Movement/DriveTrainManager.hpp"

/* CAN ID layout for drive train from a top view

          Front of Robot

         |--------------|
         |              |
      1  | M1        M2 | 2
      3  | M3        M4 | 3
      5  | M5        M6 | 6
         |              |
         |              |
         |--------------|

          Back Of Robot
*/

DriveTrain::DriveTrain(
    WPI_TalonSRX &TopLeftMotor,
    WPI_TalonSRX &TopRightMotor,

    WPI_TalonSRX &MiddleLeft,
    WPI_TalonSRX &MiddleRight,

    WPI_TalonSRX &BottomLeftMotor,
    WPI_TalonSRX &BottomRightMotor,

    // WPI_TalonSRX &HopperLift,

    FRC5572Controller &Driver,
    VisionManager &VisionManager,
    AHRS &ahrs
    ) {
        this->LeftMotors = new frc::SpeedControllerGroup( TopLeftMotor,
                                                            MiddleLeft,
                                                            BottomLeftMotor);
        this->RightMotors = new frc::SpeedControllerGroup( TopRightMotor,
                                                            MiddleRight,
                                                            BottomRightMotor);
        // this->TempRightMotors = new frc::SpeedControllerGroup( MiddleRight,
        //                                                     BottomRightMotor);
        // this->TempLeftMotors = new frc::SpeedControllerGroup( MiddleLeft,
        //                                                     BottomLeftMotor);


        this->Driver = &Driver;
        this->ahrs = &ahrs;

        this->TopLeftMotor = &TopLeftMotor;
        this->TopRightMotor = &TopRightMotor;

        this->MiddleLeft = &MiddleLeft;
        this->MiddleRight = &MiddleRight;

        this->BottomLeftMotor = &BottomLeftMotor;
        this->BottomRightMotor = &BottomRightMotor;

        // this->HopperLift = &HopperLift;

        // this->TopLeftMotorEncoder = new rev::CANEncoder{TopLeftMotor};
        // this->TopRightMotorEncoder = new rev::CANEncoder{TopRightMotor};

        // this->MiddleLeftMotorEncoder = new rev::CANEncoder{MiddleLeft};
        // this->MiddleRightMotorEncoder = new rev::CANEncoder{MiddleRight};

        // this->BottomLeftMotorEncoder = new rev::CANEncoder{BottomLeftMotor};
        // this->BottomRightMotorEncoder = new rev::CANEncoder{BottomRightMotor};

        this->LimeLight = &VisionManager;

        // DriveTrain::LowerAmps();
}

DriveTrain::~DriveTrain() {
    delete LeftMotors;
    delete RightMotors;
    delete Driver;
    delete ahrs;
}

// #define QUAD(x) (log2(x + 1) - 1)

// void DriveTrain::Drive()
// {
//     if(this->Driver->L().second > .2 || this->Driver->L().second < -.2){

//         LeftMotors->Set(-1 * QUAD(Driver->L().second) * .8  );
//     }
//     else{
//         LeftMotors->Set(0 + L);
//     }

//     if(this->Driver->R().second > .2 ||  this->Driver->R().second < -.2){
//         RightMotors->Set(QUAD(Driver->R().second)  * .8);
//     }
//     else{
//         RightMotors->Set(0 + R);
//     }
// }

void DriveTrain::Drive() {
    if (this->Driver->L().second > .2 || this->Driver->L().second < -.2) {
        LeftMotors->Set(-1 * Driver->L().second * .5);
    } else {
        LeftMotors->Set(0);
    }

    if (this->Driver->R().second > .2 ||  this->Driver->R().second < -.2) {
        RightMotors->Set(-1 * Driver->R().second  * .5);
    } else {
        RightMotors->Set(0);
    }
}

// void DriveTrain::LowerAmps() {
//     TopLeftMotor->SetSmartCurrentLimit(60);
//     TopRightMotor->SetSmartCurrentLimit(60);

//     MiddleLeft->SetSmartCurrentLimit(60);
//     MiddleRight->SetSmartCurrentLimit(60);

//     BottomLeftMotor->SetSmartCurrentLimit(60);
//     BottomRightMotor->SetSmartCurrentLimit(60);
// }

void DriveTrain::Aim() {
    if (Driver->X() ==  true) {
        disX = LimeLight->disX;
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("camMode", 0);
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("ledMode", 3);
    } else if (Driver->Y() == false) {
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("camMode", 1);
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")
                    ->PutNumber("ledMode", 1);
        disX = 0;
        T = 0;
    }

    if (fabs(disX) > 1 && Driver->X() == true) {
        if (disX > 10) {
            T = -.15;
        }
        if (disX < 10) {
            T = -disX/43;
        }
        if (disX < -10) {
            T = .15;
        }
        if (disX > -10) {
            T = disX/43;
        }
    } else {
        T = 0;
        }
}
