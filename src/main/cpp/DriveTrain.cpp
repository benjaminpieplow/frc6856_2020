/**
 * Bunch o' Code for a simple TankDrive
 */

#include <DriveTrain.h>

TankDrive::TankDrive()
{   
    //Create an array of TalonSRX objects for drive wheels, these will be addressed by the motors
    pVictorSPX[0] = new WPI_VictorSPX(1);  //Left Slave
    pTalonSRX[0] = new WPI_TalonSRX(2);  //Left Master
    pVictorSPX[1] = new WPI_VictorSPX(4);  //Right Slave
    pTalonSRX[1] = new WPI_TalonSRX(8);  //Right Master

    //Wipe Controllers
    for (int i = 0; i < 2; i++) {
        this->pVictorSPX[i]->ConfigFactoryDefault(10);
        this->pVictorSPX[i]->ConfigVoltageCompSaturation(10, 10);
        this->pTalonSRX[i]->ConfigFactoryDefault(10);
        this->pTalonSRX[i]->ConfigVoltageCompSaturation(10, 10);
        this->pVictorSPX[i]->Follow(*this->pTalonSRX[i]);
    }

}

TankDrive::~TankDrive()
{
}

void TankDrive::setTankDrivePower(double forwardSpeed, double turnSpeed)
{
    double lPower = 0.0;
    double rPower = 0.0;

    lPower = ( -forwardSpeed + turnSpeed );
    rPower = ( forwardSpeed + turnSpeed );
    this->pTalonSRX[0]->Set(ControlMode::PercentOutput, lPower);
    this->pTalonSRX[1]->Set(ControlMode::PercentOutput, rPower);
    //this->pVictorSPX[0]->Set(ControlMode::PercentOutput , lPower);
    //this->pVictorSPX[1]->Set(ControlMode::PercentOutput, rPower);

}




AdvancedDrive::AdvancedDrive(int masterCANID, int slaveCANID) {
    this->pMasterTalonSRX = new WPI_TalonSRX(masterCANID);
    this->pSlaveTalonSRX = new WPI_TalonSRX(slaveCANID);

    this->pMasterTalonSRX->ConfigFactoryDefault();
    this->pSlaveTalonSRX->ConfigFactoryDefault();

    this->pMasterTalonSRX->ConfigVoltageCompSaturation(10,10);
    this->pSlaveTalonSRX->ConfigVoltageCompSaturation(10,10);

    this->pSlaveTalonSRX->Follow(*this->pMasterTalonSRX);
}

void AdvancedDrive::InitSimpleRampedControl() {
    this->pMasterTalonSRX->ConfigVoltageCompSaturation(10, 10);
    this->pSlaveTalonSRX->ConfigVoltageCompSaturation(10, 10);
    this->pMasterTalonSRX->SetNeutralMode(NeutralMode::Brake);
    this->pSlaveTalonSRX->SetNeutralMode(NeutralMode::Brake);
    //In (seconds), ramp output power
    this->pMasterTalonSRX->ConfigOpenloopRamp(0.5);
}

void AdvancedDrive::InitCurrentControl() {
    this->pMasterTalonSRX->ConfigVoltageCompSaturation(10, 10);
    this->pSlaveTalonSRX->ConfigVoltageCompSaturation(10, 10);
    
    //Set PIDs
    this->pMasterTalonSRX->SelectProfileSlot(0, 0);
    this->pMasterTalonSRX->Config_kF(0, 1.0, 10); //LAST: 1.0     //WORKS: 1.0
    this->pMasterTalonSRX->Config_kP(0, 0.5, 10); //LAST: 0.1
    this->pMasterTalonSRX->Config_kI(0, 0.0, 10); //LAST: 0.0
    this->pMasterTalonSRX->Config_kD(0, 0.0, 10); //LAST: 0.0

    this->pMasterTalonSRX->SetNeutralMode(NeutralMode::Brake);
    this->pSlaveTalonSRX->SetNeutralMode(NeutralMode::Brake);
}


void AdvancedDrive::SetPWM(double power) {
    this->pMasterTalonSRX->Set(ControlMode::PercentOutput, power);
}

void AdvancedDrive::SetCurrent(double power) {
    this->pMasterTalonSRX->Set(ControlMode::Current, power);
}

void AdvancedDrive::VelocityTank(double joyX, double joyY) {
    //Speed (in meters per second) drivers want as maximums
    const double targetXVelocity = 1;
    const double targetYVelocity = 3;

    //Meters per Second ^2 at which the bot accelerates
    const double rampAcceleration = 2;

    
    const double encoderPulsesPerRevolution = 1440; //360 * 4
    //1 / ((Pi) * (Tire Diameter))
    const double revolutionsPerMeter = 2.12;
    //All Velocities seem to be 10 times too high, assuming 100ms sample rate for velocities
    const double sampleRateMultiplier = 0.01;
    double outputXVel = 0;

    const double rampCorrectedAcceleration = rampAcceleration * sampleRateMultiplier;

    //I'm pretty sure this sets the output in meters-per-second, no idea where the X10 comes from but it's needed
    //TODO: Best guess is that the X10 comes from the talon sampling at 1ms intervals, investigate that
    double inputXVel = targetXVelocity * joyX * revolutionsPerMeter * sampleRateMultiplier * 10;
    double inputYVel = targetYVelocity * joyY * revolutionsPerMeter * sampleRateMultiplier * 10;

    
    double deltaYVel = inputYVel - this->currentYVel;
    if ((deltaYVel) > rampCorrectedAcceleration) {
        this->currentYVel += rampCorrectedAcceleration;
    } else if (deltaYVel < -rampCorrectedAcceleration) {
        this->currentYVel += -rampCorrectedAcceleration;
    } else {
        this->currentYVel = inputYVel;
    }


    if (this->mReverseYVel == true) {
        this->pMasterTalonSRX->Set(ControlMode::Velocity, (this->currentYVel + inputXVel) * encoderPulsesPerRevolution);
    } else {
        this->pMasterTalonSRX->Set(ControlMode::Velocity, (this->currentYVel + inputXVel) * encoderPulsesPerRevolution);
    }
    
}

void AdvancedDrive::InitVelocityControl() {
    this->pMasterTalonSRX->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
    //this->pTalonSRX->ConfigSelectedFeedbackCoefficient();
    this->pMasterTalonSRX->SetSensorPhase(false);
    this->pMasterTalonSRX->SetInverted(false);
    this->pMasterTalonSRX->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    this->pMasterTalonSRX->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    //Set Maximums and Targets
    this->pMasterTalonSRX->ConfigNominalOutputForward(0, 10);
    this->pMasterTalonSRX->ConfigNominalOutputReverse(0, 10);
    this->pMasterTalonSRX->ConfigPeakOutputForward(1, 10);
    this->pMasterTalonSRX->ConfigPeakOutputReverse(-1, 10);

    //Set PIDs
    this->pMasterTalonSRX->SelectProfileSlot(0, 0);
    this->pMasterTalonSRX->Config_kF(0, 0.3, 10); //LAST: 0.3
    this->pMasterTalonSRX->Config_kP(0, 0.0, 10); //LAST: 1.6
    this->pMasterTalonSRX->Config_kI(0, 0.0, 10); //LAST: 0.0
    this->pMasterTalonSRX->Config_kD(0, 6.0, 10); //LAST: 0.0

    //Snappy Robit
//    this->pTalonSRX->Set(NeutralMode::Brake);
//    this->pVictorSPX->Set(NeutralMode::Brake);
}

void AdvancedDrive::SetTargetVelocity(double targetVel) {
    this->pMasterTalonSRX->Set(ControlMode::Velocity, targetVel);
}

void AdvancedDrive::SetTargetMotionProfileTarget(double target) {
    this->pMasterTalonSRX->Set(ControlMode::MotionMagic, target);
}

void AdvancedDrive::SetYVelocityInvert(bool invertState) {
    this->mReverseYVel = invertState;
}