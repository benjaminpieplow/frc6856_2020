/**
 * Code for actuating the lift system and preventing limit switch overrun
 */

#include <LiftSystem.h>


LiftSystem::LiftSystem(int CANID)
{
    this->m_pLiftTalon = new WPI_TalonSRX(CANID);
    
    this->m_pLiftTalon->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    this->m_pLiftTalon->SetInverted(true);
}

LiftSystem::~LiftSystem()
{
}

void LiftSystem::LiftRaise() {
    this->m_pLiftTalon->Set(ControlMode::PercentOutput, this->liftRaisePower);
}

void LiftSystem::LiftClimb() {
    this->m_pLiftTalon->Set(ControlMode::PercentOutput, this->liftLowerPower);
}

void LiftSystem::LiftStop() {
    this->m_pLiftTalon->Set(ControlMode::PercentOutput, 0);
}

void LiftSystem::LiftSystemPower(double power) {
    this->m_pLiftTalon->Set(ControlMode::PercentOutput, power);
}