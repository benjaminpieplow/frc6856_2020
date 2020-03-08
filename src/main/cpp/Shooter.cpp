
#include <Shooter.h>

Shooter::Shooter(int CANID) {

    m_pShooterMotor = new WPI_TalonFX(CANID);

    this->m_pShooterMotor->ConfigFactoryDefault();

    this->m_pShooterMotor->ConfigVoltageCompSaturation(10,10);
    this->m_pShooterMotor->EnableVoltageCompensation(true);

    this->m_pShooterMotor->SetInverted(true);

    //Set Maximums and Targets
    this->m_pShooterMotor->ConfigNominalOutputForward(0, 10);
    this->m_pShooterMotor->ConfigNominalOutputReverse(0, 10);
    this->m_pShooterMotor->ConfigPeakOutputForward(1, 10);
    this->m_pShooterMotor->ConfigPeakOutputReverse(-1, 10);

    //Set PIDs
    /**
     * TODO: Config_kF MUST be tuned to accelerate motor to base firing speed
     * PIDF should multiply target velocity by "kF" to calculate FeedForward
     * If target shooter velocity is known in encoder units per second, kF can be calulated such that:
     * PWM = targetvel * kF
     * The linear response of DC motors' RPM to Voltage should make this work
     * Also,
     * TODO: Tune PIDs
     */
    this->m_pShooterMotor->SelectProfileSlot(0, 0);
    this->m_pShooterMotor->Config_kF(0, 0.03, 10);   //LAST: 0.3
    this->m_pShooterMotor->Config_kP(0, 0.05, 10); //LAST: 0.05
    this->m_pShooterMotor->Config_kI(0, 0.001, 10); //LAST: 0.001
    this->m_pShooterMotor->Config_kD(0, 1.2, 10); //LAST: 1.2

    this->m_pShooterMotor->ConfigMaxIntegralAccumulator(0, 100000, 10);

//    this->m_pShooterMotor->Config_IntegralZone(0, 500, 10);
//    this->m_pShooterMotor->Config

//    this->m_pShooterMotor->ConfigClosedloopRamp(0.10);
}



void Shooter::EnableShooter(double targetRPM) {
    this->mShooterTargetRPM = targetRPM;
    this->EnableShooter();
}

void Shooter::EnableShooter() {
    double targetVelocity = (this->mShooterTargetRPM * this->encoderUnitsPerRevolution) / (60 * this->mSampleRateModifier);
    this->m_pShooterMotor->Set(ControlMode::Velocity, targetVelocity);
}

void Shooter::DisableShooter() {
    this->m_pShooterMotor->Set(ControlMode::PercentOutput, 0);
    //Hopefully one day
//    this->m_pShooterMotor->SetNeutralMode(Brake);
}

void Shooter::ShooterPower(double power) {
    this->m_pShooterMotor->Set(ControlMode::PercentOutput, power);
}

double Shooter::GetShooterRPM() {
    return this->mShooterTargetRPM;
}


/**
 * Inverts pixel from SmartDashboard because OpenCV is hot garbage
 */
double Shooter::GetRawYPixel() {
    double visionTargetYLowPos = frc::SmartDashboard::GetNumber("visionTargetYLowPos", -1);
    //Needs special treatment because OpenCV sends distance from the top
    if (visionTargetYLowPos > 0) {
    //If we have accurate data
        return (this->mCameraYRes - visionTargetYLowPos);
    } else {
    //If we have bad data
        return -1;
    }
}

double Shooter::GetFOVYFactor() {
    double rawYPixel = this->GetRawYPixel();
    return rawYPixel / this->mCameraYRes;
}

/**
 * Use the provided pixel to calculate the ratio of its position on the camera frame (from 0-1)
 */
double Shooter::GetFOVYFactor(double rawYPixel) {
    return rawYPixel / this->mCameraYRes;
}

/**
 * Use the latest pixel data to calculate the angle of the target relative to the camera
 */
double Shooter::GetFOVYAngle() {
    double yFactor = this->GetFOVYFactor();
    return yFactor * this->mCameraYFOV;
}

/**
 * Use the provided pixel data to calculate the angle of the target relative to the camera
 */
double Shooter::GetFOVYAngle(double yFactor) {
    return yFactor * this->mCameraYFOV;
}

/**
 * Call GetFOVYAngle and use Trig to calculate the distance of the turret from the target
 */
double Shooter::GetTargetDistance() {
    //Get the angle of the target
    double targetAngle = this->GetFOVYAngle() + this->mCameraMountAngle;
    //Convert to Radians
    double targetRadians = targetAngle * (M_PI/180);
    //Calculate distance
    double targetDistance = this->mTargetCameraElevation / tan(targetRadians);


    return targetDistance;
}

/**
 * Returns the RPM needed to sink a ball into a target at the distance provided via GetTargetDistance
 * TODO: Use the right number
 */
double Shooter::GetTargetCalculatedRPM() {
    double targetDistance = this->GetTargetDistance();
    double calculatedRPM = 206.5 * (pow(targetDistance, 2.0)) - 772.4 * targetDistance + 4585.4;
    return calculatedRPM;
}


//Is the shooter ready?
bool Shooter::ShooterReady() {
    return (!(this->ShooterUnderspeed() || this->ShooterOverspeed()));
}
//Is the shooter underspeed?
bool Shooter::ShooterUnderspeed() {
    //Convert from Encoder Ticks per 100ms to RPM
    double measuredRPM = (this->m_pShooterMotor->GetSelectedSensorVelocity(0) * 600) / this->encoderUnitsPerRevolution;
    return (measuredRPM <= (1 - this->mAllowedErrorMargin) * this->mShooterTargetRPM);
}
//Is the shooter overspeed?
bool Shooter::ShooterOverspeed() {
    //Convert from Encoder Ticks per 100ms to RPM
    double measuredRPM = (this->m_pShooterMotor->GetSelectedSensorVelocity(0) * 600) / this->encoderUnitsPerRevolution;
    return (measuredRPM >= (1 + this->mAllowedErrorMargin) * this->mShooterTargetRPM);
}

/**
 * Sets the shooter RPM to that needed to hit the target based on LemonLight's data
 * Returns true when ready to fire
 * TODO: Use a ratio between distance and firing speed that works
 */
bool Shooter::AutoRPM() {
    if (this->GetRawYPixel() > 0) {
    //If we're getting live data
        //Set the required RPM
        this->EnableShooter(this->GetTargetCalculatedRPM());

        if (this->ShooterReady()) {
        //If the shooter is at the desired RPM,
            return true;
        } else {
        //If it's still adjusting speed
            return false;
        }
    } else {
    //If there's no live data
        //Shut off the shooter
        this->DisableShooter();
        return false;
    }
}

void Shooter::ToggleAutoRPM(bool enableButton, bool disableButton) {
    if (enableButton) {
        this->mAutoRPMEnabled = true;
    }
    if (disableButton) {
        this->mAutoRPMEnabled = false;
    }
    if (this->mAutoRPMEnabled) {
        this->AutoRPM();
    }
    else
    {
        this->DisableShooter();
    }
    
}