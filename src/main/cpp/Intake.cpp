#include <Intake.h>

Intake::Intake(){
    //TODO: Assign number & Motor Controller
    this->m_pIntakeMotor = new WPI_TalonSRX(18);

    this->m_pIntakeMotor->ConfigFactoryDefault();

    this->m_pIntakeMotor->Set(NeutralMode::Brake);
    this->m_pIntakeMotor->ConfigVoltageCompSaturation(10, 10);
    this->m_pIntakeMotor->EnableVoltageCompensation(Enable);
}

//SubSystem Interactions
//Set intake to aquire balls
void Intake::ActivateIntake() {
    this->IntakeForward();
    this->LowerIntake();
}

//Set intake to standby (do not retract)
void Intake::StopIntake() {
    this->IntakeOff();
}

//Stop and stow intake
void Intake::StowIntake() {
    this->IntakeOff();
    this->RaiseIntake();
}

void Intake::ReverseIntake() {
    this->IntakeReverse();
}

//Pneumatic Actions
void Intake::RaiseIntake() {
    //Asking the right questions
}
void Intake::LowerIntake() {
    //What an interesting hypothesis
}

//Motor Action
void Intake::IntakeForward() {
    this->m_pIntakeMotor->Set(ControlMode::PercentOutput, this->mIntakePower);
}
void Intake::IntakeReverse() {
    this->m_pIntakeMotor->Set(ControlMode::PercentOutput, -1);
}

void Intake::IntakeOff() {
    this->m_pIntakeMotor->Set(ControlMode::PercentOutput, 0);
}
