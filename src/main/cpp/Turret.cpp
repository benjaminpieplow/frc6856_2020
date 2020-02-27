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
    this->m_pTurretServo->Config_kF(0, 0.0, 10); //LAST: 0.0
    this->m_pTurretServo->Config_kP(0, 0.0, 10); //LAST: 0.0
    this->m_pTurretServo->Config_kI(0, 0.0, 10); //LAST: 0.0
    this->m_pTurretServo->Config_kD(0, 0.0, 10); //LAST: 0.0

    //Motion Profile Properties in case we get an encoder
    this->m_pTurretServo->ConfigMotionCruiseVelocity(100, 10);
    this->m_pTurretServo->ConfigMotionAcceleration(100, 0);

    //A solid 2 hours of work went into this line because yours truly did not realize it was an overloaded function
    this->m_pTurretServo->ConfigForwardLimitSwitchSource(RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteTalonSRX, LimitSwitchNormal_NormallyClosed, 40, 0);
    this->m_pTurretServo->ConfigReverseLimitSwitchSource(RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteTalonSRX, LimitSwitchNormal_NormallyClosed, 40, 0);
}

/**
 * Pull the latest center-of-target pixel from NetworkTables
 */
double Turret::GetRawXPixel() {
    return frc::SmartDashboard::GetNumber("visionTargetXPos", -1);
}

/**
 * Get the latest target pixel, then calculate the ratio of its position on the camera frame
 */
double Turret::GetFOVXFactor() {
    double rawXPixel = this->GetRawXPixel();
    return rawXPixel * this->mCameraFOVXResRatio - 1;
}


/**
 * Use the provided pixel to calculate the ratio of its position on the camera frame
 */
double Turret::GetFOVXFactor(double rawXPixel) {
    return rawXPixel * this->mCameraFOVXResRatio - 1;
}

/**
 * Use the latest pixel data to calculate the angle of the target relative to the camera
 */
double Turret::GetFOVXAngle() {
    double xFactor = this->GetFOVXFactor();
    return xFactor * this->mCameraXFOV;
}

/**
 * Use the provided pixel data to calculate the angle of the target relative to the camera
 */
double Turret::GetFOVXAngle(double xFactor) {
    return xFactor * this->mCameraXFOV;
}

/**
 * Calculates the turret's angle from frame center
 */
double Turret::GetTurretFrameAngle() {
    return this->m_pTurretServo->GetSelectedSensorPosition() / this->mEncoderTicksPerDegree;
}


/**
 * Combines the turret-frame and target-turret angles
 */
double Turret::GetTargetFrameAngle() {
    return this->GetTurretFrameAngle() + this->GetFOVXAngle();
}


/**
 * Returns TRUE when GetFOVXAngle is within tolerable error margin
 */
bool Turret::GetTurretLocked() {
    return (this->GetFOVXAngle < this->mAllowableTurretError);
}

/**
 * Checks the limit switch sticky flags on the servo and the Homed object
 * If any of them are awry, flag the system as not-homed and return false
 * Otherwise, return true
 */
bool Turret::GetHomed() {
    //If homing, let RunAutoHome handle this
    if (m_Homing) {
        this->RunAutoHome();
        return this->m_Homed;
    }

    //If we're lost, find the way
    if (!this->m_Homed) {
        this->RunAutoHome();
        return this->m_Homed;
    }
    
    //If not homing, check to see if something has gone wrong
    //Get Sticky Faults from Servo Talon
    ctre::phoenix::motorcontrol::StickyFaults stickies;
    this->m_pTurretServo->GetStickyFaults(stickies);

    //If either of the limit switches have been hit, we're lost
    if (stickies.ForwardLimitSwitch || stickies.ReverseLimitSwitch) {
        //Set the system to lost
        this->m_Homed = false;
        //Start working on a fix
        this->RunAutoHome();
        //Tell the caller that we're lost (RunAutoHome should have set to false)
        return this->m_Homed;
    }

    //If none of the above returns have run, we should be in the clear!
    return this->m_Homed;
    
}


/**
 * If lost, run the mechanism towards the homing limit switch
 * If at the homing limit switch, set the encoder and set faults to be cleared
 * TODO: Finish this?
 */
void Turret::RunAutoHome() {
    
}


/**
 * If turret is homed, converts from target angle to target ticks, then pushes to Talon servo
 * If not, runs homing
 */
void Turret::SetTurretAngle(double requestAngle) {

    if (this->GetHomed()) {

    }
    double absoluteAngle = requestAngle + 90 + mHomeFrameOffset;
    double absolutePosition = absoluteAngle * this->mEncoderTicksPerDegree;
    this->m_pTurretServo->Set(ControlMode::MotionMagic, absolutePosition);
}

/**
 * Sets a percentoutput on the servo motor
 */
void Turret::SetTurretPower(double power) {
    this->m_pTurretServo->Set(ControlMode::PercentOutput, power);
}