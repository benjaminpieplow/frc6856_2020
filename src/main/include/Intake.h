/**
 * Code for the intake mechanism's motors and pneumatics
 * TODO: Build it
 * TODO: Code it
 * TODO: Map controls to actions
 */

#pragma once


#include <Pneumatics.h>
#include <ctre/Phoenix.h>

class Intake {
    public:
    Intake();
    ~Intake();

    //SubSystem Interactions
    void ActivateIntake();
    void StopIntake();
    void StowIntake();

    //Pneumatic Actions
    void RaiseIntake();
    void LowerIntake();

    //Motor Action
    void IntakeForward();
    void IntakeReverse();
    void IntakeOff();

    private:
    double mIntakePower = 0.25;

    WPI_TalonSRX* m_pIntakeMotor;

    PneumaticActuator* m_pIntakeArmActuator;
    
};