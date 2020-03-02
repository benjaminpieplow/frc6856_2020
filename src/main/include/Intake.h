/**
 * Code for the intake mechanism's motors
 * TODO: Build it
 * TODO: Code it
 * TODO: Map controls to actions
 */

#pragma once


#include <ctre/Phoenix.h>

class Intake {
    public:
    Intake(int armCANID);
    ~Intake();

    //SubSystem Interactions
    void ActivateIntake();
    void StopIntake();
    void StowIntake();
    void ReverseIntake();

    private:
    double mIntakePower = 0.25;

    WPI_TalonSRX* m_pIntakeMotor;

    //Actuation Actions
    void RaiseIntake();
    void LowerIntake();

    //Motor Action
    void IntakeForward();
    void IntakeReverse();
    void IntakeOff();
    
};