#include "Movement/Intake.hpp"

void Intake::InitPID() {
    /**
     *  The RestoreFactoryDefaults method can be used to reset the configuration parameters
     *  in the SPARK MAX to their factory default state. If no argument is passed, these
     *  parameters will not persist between power cycles
     */
    intakeMotor->RestoreFactoryDefaults();

    // intakeMotor->SetInverted(true);

    //  set PID coefficients
    m_pidController->SetP(kP);
    m_pidController->SetI(kI);
    m_pidController->SetD(kD);
    m_pidController->SetIZone(kIz);
    m_pidController->SetFF(kFF);
    m_pidController->SetOutputRange(kMinOutput, kMaxOutput);

    m_pidController2->SetP(kP);
    m_pidController2->SetI(kI);
    m_pidController2->SetD(kD);
    m_pidController2->SetIZone(kIz);
    m_pidController2->SetFF(kFF);
    m_pidController2->SetOutputRange(kMinOutput, kMaxOutput);

    //  display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
    frc::SmartDashboard::PutNumber("SetPoint", SetP);
    frc::SmartDashboard::PutNumber("Auto Selection", AutoSelection);
}

void Intake::AutoPID() {
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double SetPoint = frc::SmartDashboard::GetNumber("SetPoint", 0);
    double Auto = frc::SmartDashboard::GetNumber("Auto Selection", 0);

        //  if PID coefficients on SmartDashboard have changed,
        //  write new values to controller
        if ((p != kP)) {
            m_pidController->SetP(p);
            m_pidController2->SetP(p); kP = p;
        }
        if ((i != kI)) {
            m_pidController->SetI(i);
            m_pidController2->SetI(i); kI = i;
        }
        if ((d != kD)) {
            m_pidController->SetD(d);
            m_pidController2->SetD(d); kD = d;
        }
        if ((iz != kIz)) {
            m_pidController->SetIZone(iz);
            m_pidController2->SetIZone(iz); kIz = iz;
        }
        if ((ff != kFF)) {
            m_pidController->SetFF(ff);
            m_pidController2->SetFF(ff); kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            m_pidController->SetOutputRange(min, max);
            m_pidController2->SetOutputRange(min, max);
            kMinOutput = min; kMaxOutput = max;
        }

    if (Auto == 0) {
        //  3375
        SetPoint = 2700;
        SP = SetPoint;
        m_pidController->SetReference(SetPoint,
        rev::ControlType::kVelocity);
        intakeRPM = intakeMotorEncoder->GetVelocity();
        rpm = (intakeRPM);
    } else {
        SetPoint = 0;
    }
    frc::SmartDashboard::PutNumber("SetPoint", SetPoint);
    frc::SmartDashboard::PutNumber("ProcessVariable",
    intakeMotorEncoder->GetVelocity());
    frc::SmartDashboard::PutNumber("RPM", rpm);
    frc::SmartDashboard::PutNumber("Left RPM", intakeRPM);
}

void Intake::RunPID() {
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double SetPoint = frc::SmartDashboard::GetNumber("SetPoint", 0);

        //  if PID coefficients on SmartDashboard have changed,
        //  write new values to controller
        if ((p != kP)) {
            m_pidController->SetP(p);
            m_pidController2->SetP(p); kP = p;
        }
        if ((i != kI)) {
            m_pidController->SetI(i);
            m_pidController2->SetI(i);
            kI = i;
        }
        if ((d != kD)) {
            m_pidController->SetD(d);
            m_pidController2->SetD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            m_pidController->SetIZone(iz);
            m_pidController2->SetIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            m_pidController->SetFF(ff);
            m_pidController2->SetFF(ff);
            kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            m_pidController->SetOutputRange(min, max);
            m_pidController2->SetOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }
    if (this->Operator->POV() == 0) {  //  bumber
        SetPoint = 2600;
    } else if (this->Operator->POV() == 90) {  //  init
        SetPoint = 3375;
    } else if (this->Operator->POV() == 270) {  //  trench
        SetPoint = 3800;
    } else if (Operator->POV() == 180) {  //  colorwheel
        SetPoint = 4500;
    } else {
        SetPoint = 0;
    }

     m_pidController->SetReference(SetPoint, rev::ControlType::kVelocity);
     m_pidController2->SetReference(SetPoint, rev::ControlType::kVelocity);

    frc::SmartDashboard::PutNumber("SetPoint", SetPoint);
    frc::SmartDashboard::PutNumber("ProcessVariable",
    intakeMotorEncoder->GetVelocity());


    intakeRPM = intakeMotorEncoder->GetVelocity();
    rpm = (intakeRPM);

    frc::SmartDashboard::PutNumber("RPM", rpm);
    frc::SmartDashboard::PutNumber("Left RPM", intakeRPM);
    }


Intake::Intake(
    rev::CANSparkMax &LeftMotor,
    rev::CANSparkMax &RightMotor,
    frc::DoubleSolenoid &Hood,
    FRC5572Controller &Operator
    ) {
    m_pidController = new rev::CANPIDController{LeftMotor};
    m_pidController2 = new rev::CANPIDController{RightMotor};


    this->intakeMotor = &RightMotor;
    this->Operator = &Operator;
    // intakeMotor = new frc::SpeedControllerGroup{ LeftMotor,
    // RightMotor};
    intakeMotorEncoder = new rev::CANEncoder{LeftMotor};
}

void Intake::Shot() {
    if (Tracked) {
        intakeMotor->Set(Operator->RT());
    } else {
        intakeMotor->Set(0);
    }
}

void Intake::Test() {
    if (Operator->LB()) {
        //  Hood->Set(frc::DoubleSolenoid::Value::kForward);
        intakeMotor->Set(.65);
    } else {
        intakeMotor->Set(0);
        //  Hood->Set(frc::DoubleSolenoid::Value::kReverse);
    }
}

void Intake::TestRPM() {
    if (this->Operator->POV() == 0) {
        intakeMotor->Set(.65);
    } else if (this->Operator->POV() == 90) {
        intakeMotor->Set(.78);
    } else if (this->Operator->POV() == 270) {
        //  small adjustment from .92 to .94
        intakeMotor->Set(.90);
    } else {
        intakeMotor->Set(0.0);
    }
}

void Intake::Shots() {
    if (this->Operator->Y() == true) {
        intakeMotor->Set(.47);
    } else {
        intakeMotor->Set(0.0);
    }

    intakeRPM = intakeMotorEncoder->GetVelocity();
    rpm = (intakeRPM);

    frc::SmartDashboard::PutNumber("RPM", rpm);
    frc::SmartDashboard::PutNumber("Left RPM", intakeRPM);
}
