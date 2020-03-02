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
    this->m_pTurretServo->ConfigClearPositionOnLimitF(true, 10);
    this->m_pTurretServo->ConfigClearPositionOnLimitR(false, 10);

    //Set Maximums and Targets
    this->m_pTurretServo->ConfigNominalOutputForward(0, 10);
    this->m_pTurretServo->ConfigNominalOutputReverse(0, 10);
    this->m_pTurretServo->ConfigPeakOutputForward(0.10, 10);
    this->m_pTurretServo->ConfigPeakOutputReverse(-0.10, 10);

    //Set PIDs
    this->m_pTurretServo->SelectProfileSlot(0, 0);
    this->m_pTurretServo->Config_kF(0, 0, 10); //LAST: 0.0
    this->m_pTurretServo->Config_kP(0, 0.8, 10); //LAST: 0.0
    this->m_pTurretServo->Config_kI(0, 0.0, 10); //LAST: 0.0
    this->m_pTurretServo->Config_kD(0, 0.0, 10); //LAST: 0.0

    //Motion Profile Properties to avoid jerky movements
    this->m_pTurretServo->ConfigMotionCruiseVelocity(200, 10);
    this->m_pTurretServo->ConfigMotionAcceleration(100, 0);
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
    return (this->GetFOVXAngle() < this->mAllowableTurretError);
}


bool Turret::GetHomed() {

    ctre::phoenix::motorcontrol::StickyFaults stickies;
    this->m_pTurretServo->GetStickyFaults(stickies);

    if (stickies.ForwardLimitSwitch) {
        this->SetTurretPower(-0.1);
        this->mReverseLimitTripped = false;
        return false;
    } else if (stickies.ReverseLimitSwitch || this->mReverseLimitTripped) {
        this->SetTurretPower(0.1);
        this->mReverseLimitTripped = true;
        return false;
    } else {
        this->m_pTurretServo->ClearStickyFaults();
        return true;
    }

}

/**
 * Checks the limit switch sticky flags on the servo and the Homed object
 * If any of them are awry, flag the system as not-homed and return false
 * Otherwise, return true
 
bool Turret::GetHomed() {
    //If homing, let RunAutoHome handle this
    if (mHoming) {
        this->RunAutoHome();
        return false;
    }

    //If we're lost, find the way
    if (!(this->mHomed)) {
        this->RunAutoHome();
        return false;
    }
    
    //If not homing, check to see if something has gone wrong
    //Get Sticky Faults from Servo Talon
    ctre::phoenix::motorcontrol::StickyFaults stickies;
    this->m_pTurretServo->GetStickyFaults(stickies);

    //If either of the limit switches have been hit, we're lost
    if (stickies.ReverseLimitSwitch) {
        //Set the system to lost
        this->mHomed = false;
        //Start working on a fix
        this->RunAutoHome();
        //Tell the caller that we're lost (RunAutoHome should have set to false)
        return this->mHomed;
    }
    if (stickies.ForwardLimitSwitch) {
        this->RunAutoHome();
        return false;
    }

    //TODO: Driver Override

    //If none of the above returns have run, we should be in the clear!
    return this->mHomed;
    
}
*/

/**
 * If lost, run the mechanism towards the homing limit switch
 * If at the homing limit switch, set the encoder and set faults to be cleared
 * If moved away from homing limit switch, set homed
 
void Turret::RunAutoHome() {

    
    //Get Sticky Faults from Servo Talon
    ctre::phoenix::motorcontrol::StickyFaults stickies;
    this->m_pTurretServo->GetStickyFaults(stickies);

    //If we're coming off the limit switch, clear limits 
    if (mZeroHit && !stickies.ForwardLimitSwitch) {
        this->m_pTurretServo->ClearStickyFaults(10);
        this->mHomed = true;
        this->mHoming = false;
        this->mZeroHit = false;
        this->SetTurretAngle(0);
        frc::SmartDashboard::PutBoolean("DB/LED 3", true);
        
    }
    //If we are on the home limit switch
    else if (stickies.ForwardLimitSwitch) {
        //Fly away from the home switch
        this->SetTurretPower(-0.1);
        //Forget the bad times (check on next iteration)
        this->m_pTurretServo->ClearStickyFaults();
        //Update Flags
        this->mHomed = false;
        this->mHoming = false;
        this->mZeroHit = true;
        frc::SmartDashboard::PutBoolean("DB/LED 2", true);
    }
    //If we are not on the home limit switch
    else {
        //Fly towards the home switch
        this->SetTurretPower(0.1);
        //Clear the faults so we can check again next time
        this->m_pTurretServo->ClearStickyFaults();
        //Update Flags
        this->mHomed = false;
        this->mHoming = true;
        frc::SmartDashboard::PutBoolean("DB/LED 1", true);
    }
}
*/

/**
 * If turret is homed, converts from target angle to target ticks, then pushes to Talon servo
 * If not, runs homing
 */
void Turret::SetTurretAngle(double requestAngle) {

    //TODO: if (this->GetHomed())

    double targetEncoderPosition = requestAngle * this->mEncoderTicksPerDegree + this->mHomeFrameOffset * this->mEncoderTicksPerDegree;

    this->m_pTurretServo->Set(ControlMode::MotionMagic, targetEncoderPosition);
}

/**
 * Sets a percentoutput on the servo motor
 * Used for homing and testing, shouldn't be
 */
void Turret::SetTurretPower(double power) {
    this->m_pTurretServo->Set(ControlMode::PercentOutput, power);
}