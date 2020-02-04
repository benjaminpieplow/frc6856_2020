
#include <Shooter.h>

Shooter::Shooter(int CANID) {

    pShooterMotor = new WPI_TalonSRX(CANID);

    this->pShooterMotor->ConfigFactoryDefault();

    this->pShooterMotor->ConfigVoltageCompSaturation(10,10);

    this->pShooterMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
    this->pShooterMotor->SetSensorPhase(false);
    this->pShooterMotor->SetInverted(false);
    this->pShooterMotor->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    this->pShooterMotor->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    //Set Maximums and Targets
    this->pShooterMotor->ConfigNominalOutputForward(0, 10);
    this->pShooterMotor->ConfigNominalOutputReverse(0, 10);
    this->pShooterMotor->ConfigPeakOutputForward(1, 10);
    this->pShooterMotor->ConfigPeakOutputReverse(-1, 10);

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
    this->pShooterMotor->SelectProfileSlot(0, 0);
    this->pShooterMotor->Config_kF(0, 0.3, 10);
    this->pShooterMotor->Config_kP(0, 0.1, 10); //LAST: 0.1
    this->pShooterMotor->Config_kI(0, 0.0, 10); //LAST: 0.0
    this->pShooterMotor->Config_kD(0, 0.0, 10); //LAST: 0.0

    //Set Accel and Cruise Velocity
    //Used for Motion Magic, is worth investigating if slamming to RPM angers the integral accumulator, will disable for now
    //this->pShooterMotor->ConfigMotionCruiseVelocity(500, 10);
    //this->pShooterMotor->ConfigMotionAcceleration(1500, 10);

    //Get that little bit of juice back
    this->pShooterMotor->Set(NeutralMode::Brake);

}

void Shooter::EnableShooter() {
    this->pShooterEnabled = true;
}

void Shooter::SetTargetVelocity(double targetRPM) {
    this->pShooterTargetRPM = targetRPM;
    this->SetTargetVelocity();
}

void Shooter::SetTargetVelocity() {
    double targetVelocity = (this->pShooterTargetRPM * this->encoderUnitsPerRevolution) / 60;
    this->pShooterMotor->Set(ControlMode::Velocity, targetVelocity);
}