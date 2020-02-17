
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
    this->m_pShooterMotor->Config_kF(0, 0.3, 10);
    this->m_pShooterMotor->Config_kP(0, 0.01, 10); //LAST: 0.04
    this->m_pShooterMotor->Config_kI(0, 0.0005, 10); //LAST: 0.0
    this->m_pShooterMotor->Config_kD(0, 0.6, 10); //LAST: 0.0

    this->m_pShooterMotor->Config_IntegralZone(0, 500, 10);

    this->m_pShooterMotor->ConfigClosedloopRamp(0.10);
}



void Shooter::EnableShooter(double targetRPM) {
    this->mShooterTargetRPM = targetRPM;
    this->EnableShooter();
}

void Shooter::EnableShooter() {
    double targetVelocity = (this->mShooterTargetRPM * this->encoderUnitsPerRevolution) / 600;
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