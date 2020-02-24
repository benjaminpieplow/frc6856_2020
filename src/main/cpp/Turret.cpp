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

    //A solid 2 hours of work went into this line because yours truly did not realize it was an overloaded function
    this->m_pTurretServo->ConfigForwardLimitSwitchSource(RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteTalonSRX, LimitSwitchNormal_NormallyClosed, 40, 0);
    this->m_pTurretServo->ConfigReverseLimitSwitchSource(RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteTalonSRX, LimitSwitchNormal_NormallyClosed, 40, 0);
    

}

//Returns the angle of the target relative to the turret in degrees
double Turret::GetFOVTargetXAngle() {
    this->pCameraTargetFOVXAngle = this->pCameraXFOV * this->pCameraTargetFOVXAngle;
    return this->pCameraTargetFOVXAngle;
}
//Converts from FOVTargetFactor to FOVTargetAngle
double Turret::GetFOVTargetAngle(double FOVTargetFactor) {
    this->pCameraTargetFOVXAngle = this->pCameraXFOV * FOVTargetFactor;
    return this->pCameraTargetFOVXAngle;
}

//Returns the angle of the target relative to the turret as a factor of the camera's FOV (-1 to +1)
double Turret::GetFOVTargetFactor() {
    this->pCameraTargetFOVXFactor = this->pCameraFOVResRatio * this->pCameraTargetHPos;
    return this->pCameraTargetFOVXFactor;
}
//Converts from hPos to FOVTargetFactor
double Turret::GetFOVTargetFactor(double hPos) {
    this->pCameraTargetFOVXFactor = this->pCameraFOVResRatio * hPos;
    return this->pCameraTargetFOVXFactor;
}

//Returns the angle of the target relative to the frame
double Turret::GetFrameTargetAngle() {
    /**
     * TODO:
     */
}

void Turret::SetTurretPower(double power) {
    this->m_pTurretServo->Set(ControlMode::PercentOutput, power);
}

void Turret::SetTurretAngle(double angle) {
    
}