
#include <Shooter.h>

Shooter::Shooter(int CANID) {

    m_pShooterMotor = new WPI_TalonSRX(CANID);

    //Clear controller
    this->m_pShooterMotor->ConfigFactoryDefault();

    //Unifies robot behavior across battery levels
    this->m_pShooterMotor->ConfigVoltageCompSaturation(10,10);

    //Setup encoder
    this->m_pShooterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
    this->m_pShooterMotor->SetSensorPhase(false);
    this->m_pShooterMotor->SetInverted(false);
    //Sync Talon and Rio update speeds (note: affacts CAN usage)
    this->m_pShooterMotor->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    this->m_pShooterMotor->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

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
    this->m_pShooterMotor->Config_kF(0, 0.3, 10);
    this->m_pShooterMotor->Config_kP(0, 0.1, 10); //LAST: 0.1
    this->m_pShooterMotor->Config_kI(0, 0.0, 10); //LAST: 0.0
    this->m_pShooterMotor->Config_kD(0, 0.0, 10); //LAST: 0.0

    /**
     * Set Accel and Cruise Velocity
     * Used for Motion Magic, is worth investigating if slamming to RPM angers the integral accumulator, will disable for now
     * Also, consider iterative ramp like that in VelocityTank
     */
    //this->m_pShooterMotor->ConfigMotionCruiseVelocity(500, 10);
    //this->m_pShooterMotor->ConfigMotionAcceleration(1500, 10);

    //Get that little bit of juice back
    this->m_pShooterMotor->Set(NeutralMode::Brake);

}

/**
 * Currently sets the shooter to run
 * In the future, should enable shooter motor and trigger a velocity ramp
 */
void Shooter::EnableShooter() {
    this->mShooterEnabled = true;
}

/**
 * Sets the target velocity (in RPM) of the OBJECT'S VELOCITY SETPOINT
 * Runs the code to relay that setpoint to the shooter Talon
 */
void Shooter::SetTargetVelocity(double targetRPM) {
    this->mShooterTargetRPM = targetRPM;
    this->SetTargetVelocity();
}

/**
 * Tells the shooter talon what speed to yeet at
 */
void Shooter::SetTargetVelocity() {
    double targetVelocity = (this->mShooterTargetRPM * this->encoderUnitsPerRevolution) / 60;
    this->m_pShooterMotor->Set(ControlMode::Velocity, targetVelocity);
}