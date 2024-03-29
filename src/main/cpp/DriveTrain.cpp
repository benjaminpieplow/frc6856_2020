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




AdvancedDrive::AdvancedDrive(int masterCANID, int slaveCANID, bool inverted) {
    this->pMasterTalonSRX = new WPI_TalonSRX(masterCANID);
    this->pSlaveTalonSRX = new WPI_TalonSRX(slaveCANID);

    this->pMasterTalonSRX->ConfigFactoryDefault();
    this->pSlaveTalonSRX->ConfigFactoryDefault();

    this->pMasterTalonSRX->ConfigVoltageCompSaturation(10,10);
    this->pSlaveTalonSRX->ConfigVoltageCompSaturation(10,10);

    this->pSlaveTalonSRX->Follow(*this->pMasterTalonSRX);

    if (inverted) {
        this->pMasterTalonSRX->SetInverted(true);
        this->pSlaveTalonSRX->SetInverted(true);
    }
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


/**
 * Same as above, but uses a trigger for extra BOOST
 */
void AdvancedDrive::VelocityTank(double joyX, double joyY, double joyBoost) {
    //If power is under 5%, zero stick inputs
    //Done to prevent integral windup
    if (!(-0.05 > joyX || joyX > 0.05 || -0.05 > joyY || joyY > 0.05)) {
        joyX = 0;
        joyY = 0;
    }

    //Speed (in meters per second) drivers want as maximums
    const double targetXVelocity = 1;
    const double targetYVelocity = 2;
    const double targetBoostVelocity = 2;
    const double boostSteerReductionFactor = 0.5;

    //Meters per Second at which the bot accelerates
    const double rampAcceleration = 1;

    //Number of encoder units per rotation of the output shaft
    const double encoderPulsesPerRevolution = 1440; //360 * 4
    //1 / ((Pi) * (Tire Diameter))
    const double revolutionsPerMeter = 2.12;
    //All Velocities seem to be 10 times too high, assuming 100ms sample rate for velocities
    const double sampleRateMultiplier = 0.01;
    const double rampCorrectedAcceleration = rampAcceleration * sampleRateMultiplier;

    //Encoder units per second sent to Talon
    double outputXVel = 0;

    //I'm pretty sure this sets the output in meters-per-second, no idea where the X10 comes from but it's needed
    double inputXVel = targetXVelocity * joyX * revolutionsPerMeter * sampleRateMultiplier * 10 - joyBoost * targetXVelocity * joyX * revolutionsPerMeter * sampleRateMultiplier * boostSteerReductionFactor * 10;
    double inputYVel = targetYVelocity * joyY * revolutionsPerMeter * sampleRateMultiplier * 10 + targetBoostVelocity * joyY * joyBoost * revolutionsPerMeter * sampleRateMultiplier * 10;

    //Ramp
    //TODO: Rapid Decay (or maybe not, this robit stops pretty quickly)

    double deltaYVel = inputYVel - this->currentYVel;
    if ((deltaYVel) > rampCorrectedAcceleration) {
    //If the new velocity is too great of a change in the positive
        //Add only a small change
        this->currentYVel += rampCorrectedAcceleration;
    } else if (deltaYVel < -rampCorrectedAcceleration) {
    //If the new velocity is too great of a change in the negative
        //Add only a small change
        this->currentYVel += -rampCorrectedAcceleration;
    } else {
    //If the new velocity is just right
        //Make it so
        this->currentYVel = inputYVel;
    }


    if (this->mReverseYVel == true) {
        this->pMasterTalonSRX->Set(ControlMode::Velocity, (inputXVel - this->currentYVel) * encoderPulsesPerRevolution);
    } else {
        this->pMasterTalonSRX->Set(ControlMode::Velocity, (inputXVel + this->currentYVel) * encoderPulsesPerRevolution);
    }
    
    
}

void AdvancedDrive::InitVelocityControl() {
    this->pMasterTalonSRX->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
    //this->pTalonSRX->ConfigSelectedFeedbackCoefficient();
    this->pMasterTalonSRX->SetSensorPhase(true);
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
    this->pMasterTalonSRX->Config_kP(0, 3.2, 10); //LAST: 1.6
    this->pMasterTalonSRX->Config_kI(0, 0.01, 10); //LAST: 0.0
    this->pMasterTalonSRX->Config_kD(0, 6.0, 10); //LAST: 0.0

    this->pMasterTalonSRX->ConfigMotionCruiseVelocity(500, 10);
    this->pMasterTalonSRX->ConfigMotionAcceleration(200, 10);
}

void AdvancedDrive::SetTargetVelocity(double targetVel) {
    this->pMasterTalonSRX->Set(ControlMode::Velocity, targetVel);
}

void AdvancedDrive::SetTargetMotionProfileTarget(double target) {
    //TODO: Switch constants over to private members
    //Number of encoder units per rotation of the output shaft
    const double encoderPulsesPerRevolution = 1440; //360 * 4
    //1 / ((Pi) * (Tire Diameter))
    const double revolutionsPerMeter = 2.12;

    double targetEncoderUnits = target * encoderPulsesPerRevolution * revolutionsPerMeter;
    this->pMasterTalonSRX->Set(ControlMode::MotionMagic, targetEncoderUnits);
}

void AdvancedDrive::SetYVelocityInvert(bool invertState) {
    this->mReverseYVel = invertState;
}

void AdvancedDrive::ZeroEncoder() {
    this->pMasterTalonSRX->SetSelectedSensorPosition(0, 0);
}