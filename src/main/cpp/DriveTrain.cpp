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




AdvancedDrive::AdvancedDrive(int talonCANID, int victorCANID) {
    this->pTalonSRX = new WPI_TalonSRX(talonCANID);
    this->pVictorSPX = new WPI_VictorSPX(victorCANID);

    this->pTalonSRX->ConfigFactoryDefault();
    this->pVictorSPX->ConfigFactoryDefault();

    this->pTalonSRX->ConfigVoltageCompSaturation(10,10);
    this->pVictorSPX->ConfigVoltageCompSaturation(10,10);

    this->pVictorSPX->Follow(*this->pTalonSRX);
}

void AdvancedDrive::InitSimpleRampedControl() {
    this->pTalonSRX->ConfigVoltageCompSaturation(10, 10);
    this->pVictorSPX->ConfigVoltageCompSaturation(10, 10);
    this->pTalonSRX->SetNeutralMode(NeutralMode::Brake);
    this->pVictorSPX->SetNeutralMode(NeutralMode::Brake);
    //In (seconds), ramp output power
    this->pTalonSRX->ConfigOpenloopRamp(0.5);
}

void AdvancedDrive::InitCurrentControl() {
    this->pTalonSRX->ConfigVoltageCompSaturation(10, 10);
    this->pVictorSPX->ConfigVoltageCompSaturation(10, 10);
    
    //Set PIDs
    this->pTalonSRX->SelectProfileSlot(0, 0);
    this->pTalonSRX->Config_kF(0, 1.0, 10); //LAST: 1.0     //WORKS: 1.0
    this->pTalonSRX->Config_kP(0, 0.5, 10); //LAST: 0.1
    this->pTalonSRX->Config_kI(0, 0.0, 10); //LAST: 0.0
    this->pTalonSRX->Config_kD(0, 0.0, 10); //LAST: 0.0

    //Investigate if this is causing issuese with max power
    this->pTalonSRX->SetNeutralMode(NeutralMode::Brake);
    this->pVictorSPX->SetNeutralMode(NeutralMode::Brake);
}


void AdvancedDrive::SetPWM(double power) {
    this->pTalonSRX->Set(ControlMode::PercentOutput, power);
}

void AdvancedDrive::SetCurrent(double power) {
    this->pTalonSRX->Set(ControlMode::Current, power);
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
        this->pTalonSRX->Set(ControlMode::Velocity, (inputXVel - this->currentYVel) * encoderPulsesPerRevolution);
    } else {
        this->pTalonSRX->Set(ControlMode::Velocity, (inputXVel + this->currentYVel) * encoderPulsesPerRevolution);
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
    const double rampAcceleration = 2;

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
        this->currentYVel += rampCorrectedAcceleration;
    } else if (deltaYVel < -rampCorrectedAcceleration) {
        this->currentYVel += -rampCorrectedAcceleration;
    } else {
        this->currentYVel = inputYVel;
    }


    if (this->mReverseYVel == true) {
        this->pTalonSRX->Set(ControlMode::Velocity, (inputXVel - this->currentYVel) * encoderPulsesPerRevolution);
    } else {
        this->pTalonSRX->Set(ControlMode::Velocity, (inputXVel + this->currentYVel) * encoderPulsesPerRevolution);
    }
    
    
}

void AdvancedDrive::InitVelocityControl() {
    this->pTalonSRX->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
    //this->pTalonSRX->ConfigSelectedFeedbackCoefficient();
    this->pTalonSRX->SetSensorPhase(false);
    this->pTalonSRX->SetInverted(false);
    this->pTalonSRX->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    this->pTalonSRX->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    //Set Maximums and Targets
    this->pTalonSRX->ConfigNominalOutputForward(0, 10);
    this->pTalonSRX->ConfigNominalOutputReverse(0, 10);
    this->pTalonSRX->ConfigPeakOutputForward(1, 10);
    this->pTalonSRX->ConfigPeakOutputReverse(-1, 10);

    //Set PIDs
    this->pTalonSRX->SelectProfileSlot(0, 0);
    this->pTalonSRX->Config_kF(0, 0.3, 10); //LAST: 0.3
    this->pTalonSRX->Config_kP(0, 3.2, 10); //LAST: 1.6
    this->pTalonSRX->Config_kI(0, 0.01, 10); //LAST: 0.01
    this->pTalonSRX->Config_kD(0, 6.0, 10); //LAST: 6.0

    this->pTalonSRX->ConfigMotionAcceleration(500, 10);
    this->pTalonSRX->ConfigMotionCruiseVelocity(500, 10);

}

void AdvancedDrive::SetTargetVelocity(double targetVel) {
    this->pTalonSRX->Set(ControlMode::Velocity, targetVel);
}

void AdvancedDrive::SetTargetMotionProfileTarget(double target) {
    this->pTalonSRX->Set(ControlMode::MotionMagic, target);
}

void AdvancedDrive::SetYVelocityInvert(bool invertState) {
    this->mReverseYVel = invertState;
}