/**
 * Code for the intake mechanism's motors
 * TODO: Build it
 * TODO: Code it
 * TODO: Map controls to actions
 */

#pragma once


#include <ctre/Phoenix.h>
#include <frc/Timer.h>

class IntakeArm {
    public:
    IntakeArm(int armCANID);
    ~IntakeArm();

    //Actuation Actions
    void RaiseIntake();
    void LowerIntake();
    void HoldIntake();
    void IntakePeriodic();

    //You know what is its ;)
    void SetIntakePower(double power);


    private:

    //Intake Arm Power Setpoints
    const double mIntakeRaisePower = -0.25;
    const double mIntakeHoldPower = -0.08;
    const double mIntakeLowerPower = 0.20;
    //Action Time Setpoints
    const double mIntakeRaiseTime = 2;
    const double mIntakeLowerTime = 1;

    //Action Timer
    frc::Timer mActionTimer;

    //Automation Sentinels
    double mActionStartTime = 0;
    bool mLowerLatch = false;
    bool mRaiseLatch = false;
    bool mIntakeStowed = true;


    WPI_TalonSRX* m_pIntakeMotor;

    
};