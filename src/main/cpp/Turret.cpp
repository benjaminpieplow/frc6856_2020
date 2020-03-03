/**
 * Code for the turret tracking system
 */

#include <Turret.h>

Turret::Turret(int CANID) {
    this->m_pTurretServo = new WPI_TalonSRX(CANID);

    this->m_pTurretServo->ConfigFactoryDefault();
    //Problems? Forget about them.
    this->m_pTurretServo->ClearStickyFaults();

    //Sensor Configuration
    this->m_pTurretServo->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
    this->m_pTurretServo->SetSensorPhase(true);     //Invert Input
    this->m_pTurretServo->SetInverted(false);       //Output is fine

    //A solid 2 hours of work went into this line because yours truly did not realize it was an overloaded function
    this->m_pTurretServo->ConfigForwardLimitSwitchSource(RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteTalonSRX, LimitSwitchNormal_NormallyOpen, 40, 0);
    this->m_pTurretServo->ConfigReverseLimitSwitchSource(RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteTalonSRX, LimitSwitchNormal_NormallyOpen, 40, 0);

    //When fwd limit switch hit, "ZERO" encoder
    this->m_pTurretServo->ConfigClearPositionOnLimitF(false, 10);
    this->m_pTurretServo->ConfigClearPositionOnLimitR(true, 10);

    //Set Maximums and Targets
    this->m_pTurretServo->ConfigNominalOutputForward(0, 10);
    this->m_pTurretServo->ConfigNominalOutputReverse(0, 10);
    this->m_pTurretServo->ConfigPeakOutputForward(0.25, 10);
    this->m_pTurretServo->ConfigPeakOutputReverse(-0.25, 10);
    //Soft Limits
    int forwardSoftLimit = int((this->mHomeFrameOffset + this->mSoftLimitFromCenter) * this->mEncoderTicksPerDegree);
    int reverseSoftLimit = int((this->mHomeFrameOffset - this->mSoftLimitFromCenter) * this->mEncoderTicksPerDegree);
    this->m_pTurretServo->ConfigForwardSoftLimitThreshold(forwardSoftLimit, 10);
    this->m_pTurretServo->ConfigForwardSoftLimitEnable(true, 10);
    this->m_pTurretServo->ConfigReverseSoftLimitThreshold(reverseSoftLimit, 10);
    this->m_pTurretServo->ConfigReverseSoftLimitEnable(true, 10);

    //Set PIDs
    this->m_pTurretServo->SelectProfileSlot(0, 0);
    this->m_pTurretServo->Config_kF(0, 0, 10); //LAST: 0.0
    this->m_pTurretServo->Config_kP(0, 0.8, 10); //LAST: 0.8
    this->m_pTurretServo->Config_kI(0, 0.0, 10); //LAST: 0.0
    this->m_pTurretServo->Config_kD(0, 0.0, 10); //LAST: 0.0

    //Motion Profile Properties to avoid jerky movements
    this->m_pTurretServo->ConfigMotionCruiseVelocity(400, 10);
    this->m_pTurretServo->ConfigMotionAcceleration(200, 0);
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
    return xFactor * this->mCameraXFOV / 2;
}

/**
 * Use the provided pixel data to calculate the angle of the target relative to the camera
 */
double Turret::GetFOVXAngle(double xFactor) {
    return xFactor * this->mCameraXFOV / 2;
}

/**
 * Calculates the turret's angle from frame center
 */
double Turret::GetTurretFrameAngle() {
    return this->m_pTurretServo->GetSelectedSensorPosition() / this->mEncoderTicksPerDegree - this->mHomeFrameOffset;
}


/**
 * Combines the turret-frame and target-turret angles
 */
double Turret::GetTargetFrameAngle() {
    return this->GetTurretFrameAngle() + this->GetFOVXAngle();
}


/**
 * Returns false when GetRawPixel is negative (indicating no lock from LemonLight)
 */
bool Turret::GetTargetTracked() {
    if (this->GetRawXPixel() < 0) {
        return false;
    } else
    {
        return true;
    }
    
}


/**
 * Returns TRUE when GetFOVXAngle is within tolerable error margin
 */
bool Turret::GetTurretLocked() {
    return (abs(this->GetFOVXAngle()) < this->mAllowableTurretError);
}

/**
 * Returns "true" if the turret is homed and ready for motion control
 * Returns "false" and runs the turret home if it's not
 */
bool Turret::GetHomed() {

    //Get limit switch sticky faults (stick true even during switch overruns)
    ctre::phoenix::motorcontrol::StickyFaults stickies;
    this->m_pTurretServo->GetStickyFaults(stickies);

    /** Actual Homing Code */
    if (stickies.ReverseLimitSwitch) {
    //If at home
        //Drive away from Home
        this->SetTurretPower(-0.20);
        //Clear flag if hit forward stop
        this->mForwardLimitTripped = false;
        //Clear faults to allow checking again
        this->m_pTurretServo->ClearStickyFaults();

        //Re-enable soft limits
        this->m_pTurretServo->ConfigForwardSoftLimitEnable(true, 10);
        this->m_pTurretServo->ConfigReverseSoftLimitEnable(true, 10);


        return false;
    } else if (stickies.ForwardLimitSwitch || this->mForwardLimitTripped) {
    //If far-from-home switch has been hit
        this->SetTurretPower(-0.20);
        this->mForwardLimitTripped = true;
        //Re-enable soft limits
        this->m_pTurretServo->ConfigForwardSoftLimitEnable(true, 10);
        this->m_pTurretServo->ConfigReverseSoftLimitEnable(true, 10);

        //Temporarily allow out-of-bounds running
        this->m_pTurretServo->ConfigForwardSoftLimitEnable(false, 10);
        this->m_pTurretServo->ConfigReverseSoftLimitEnable(false, 10);
        return false;
    } else {
    //If properly homed
        return true;
    }

}

/**
 * If turret is homed, converts from target angle to target ticks, then pushes to Talon servo
 * If not, runs homing
 */
void Turret::SetTurretAngle(double requestAngle) {

    if (this->GetHomed()) {
    //If we're at home,
        if (abs(requestAngle) < this->mSoftLimitFromCenter) {
        //And the request is reasonable, satisfy it
            //Set requested angle
            double targetEncoderPosition = requestAngle * this->mEncoderTicksPerDegree + this->mHomeFrameOffset * this->mEncoderTicksPerDegree;
            this->m_pTurretServo->Set(ControlMode::MotionMagic, targetEncoderPosition);
        } else {
        //If the request is outside of bounds, we have to curtail it so we don't overshoot the soft limits
            if (requestAngle > this->mSoftLimitFromCenter) {
            //If it's bigger, make it equal to max positive
                requestAngle = this->mSoftLimitFromCenter;
            } else {
            //If it's smaller, make it equal to max negative
                requestAngle = -1 * this->mSoftLimitFromCenter;
            }
            //Set Requested angle
            double targetEncoderPosition = requestAngle * this->mEncoderTicksPerDegree + this->mHomeFrameOffset * this->mEncoderTicksPerDegree;
            this->m_pTurretServo->Set(ControlMode::MotionMagic, targetEncoderPosition);
        }
        
    }
    

    
}

/**
 * Gets the target offset from frame, then servos to that position
 * If there is a valid target, goes there
 * If there is no valid target, goes to zero
 * If locked on target, returns true
 */
bool Turret::AutoTurret() {
    if (this->GetTargetTracked()) {
        this->SetTurretAngle(this->GetTargetFrameAngle());
        if (this->GetTurretLocked()) {
            return true;
        } else {
            return false;
        }
    } else {
        this->SetTurretAngle(0);
        return false;
    }

}

/**
 * Sets a percentoutput on the servo motor
 * Used for homing and testing, shouldn't be
 */
void Turret::SetTurretPower(double power) {
    this->m_pTurretServo->Set(ControlMode::PercentOutput, power);
}