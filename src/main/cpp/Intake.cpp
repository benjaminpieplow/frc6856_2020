#include <Intake.h>

Intake::Intake(){
    this->m_pIntakeArmActuator = new PneumaticActuator(1,2);
    //TODO: Assign number
    this->m_pIntakeMotor = new WPI_TalonSRX(10);

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

//Pneumatic Actions
void Intake::RaiseIntake() {
    this->m_pIntakeArmActuator->setPneumaticActuator(true);
}
void Intake::LowerIntake() {
    this->m_pIntakeArmActuator->setPneumaticActuator(false);
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
