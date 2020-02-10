/**
 * Code for the turret tracking system
 */

#include <Turret.h>

Turret::Turret(int CANID) {
    this->m_pTurretServo = new WPI_TalonSRX(CANID);

    this->m_pTurretServo->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
    this->m_pTurretServo->SetSensorPhase(false);
    this->m_pTurretServo->SetInverted(false);
    this->m_pTurretServo->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    this->m_pTurretServo->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    //Set Maximums and Targets
    this->m_pTurretServo->ConfigNominalOutputForward(0, 10);
    this->m_pTurretServo->ConfigNominalOutputReverse(0, 10);
    this->m_pTurretServo->ConfigPeakOutputForward(1, 10);
    this->m_pTurretServo->ConfigPeakOutputReverse(-1, 10);

    //Set PIDs
    this->m_pTurretServo->SelectProfileSlot(0, 0);
    this->m_pTurretServo->Config_kF(0, 0.3, 10); //LAST: 0.3
    this->m_pTurretServo->Config_kP(0, 3.2, 10); //LAST: 1.6
    this->m_pTurretServo->Config_kI(0, 0.0, 10); //LAST: 0.0
    this->m_pTurretServo->Config_kD(0, 6.0, 10); //LAST: 0.0

    this->m_pTurretServo->ConfigMotionCruiseVelocity(100, 10);
    this->m_pTurretServo->ConfigMotionAcceleration(100, 0);
}