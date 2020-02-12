/**
 * Bunch o' Code for a simple TankDrive
 */

#include <DriveTrain.h>

/**
 * Initiates a simple TankDrive
 * Contains both 'tracks' in the same object
 */
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
    //You want a destructor? Reboot the robot
}

/**
 * Sets Talons and Victors to required speed inputting joystick positions -1 through 1
 */
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



/**
 * A more complex tank drive system, this only does one track
 * Inputs CANIDs to bind motors
 */
AdvancedDrive::AdvancedDrive(int talonCANID, int victorCANID) {
    this->pTalonSRX = new WPI_TalonSRX(talonCANID);
    this->pVictorSPX = new WPI_VictorSPX(victorCANID);

    this->pTalonSRX->ConfigFactoryDefault();
    this->pVictorSPX->ConfigFactoryDefault();

    this->pTalonSRX->ConfigVoltageCompSaturation(10,10);
    this->pVictorSPX->ConfigVoltageCompSaturation(10,10);

    this->pVictorSPX->Follow(*this->pTalonSRX);
}

/**
 * Initiates simple ramped robot control
 * Performs simple VOLTAGE ramping
 * Handles like a beached barge
 * Pairs with SetPWM
 */
void AdvancedDrive::InitSimpleRampedControl() {
    this->pTalonSRX->ConfigVoltageCompSaturation(10, 10);
    this->pVictorSPX->ConfigVoltageCompSaturation(10, 10);
    this->pTalonSRX->SetNeutralMode(NeutralMode::Brake);
    this->pVictorSPX->SetNeutralMode(NeutralMode::Brake);
    //In (seconds), ramp output power
    this->pTalonSRX->ConfigOpenloopRamp(0.5);
}

/**
 * Mainly used to test torque control
 * Actually just pisses off the robot
 */
void AdvancedDrive::InitCurrentControl() {
    this->pTalonSRX->ConfigVoltageCompSaturation(10, 10);
    this->pVictorSPX->ConfigVoltageCompSaturation(10, 10);
    
    //Set PIDs
    this->pTalonSRX->SelectProfileSlot(0, 0);
    this->pTalonSRX->Config_kF(0, 1.0, 10); //LAST: 1.0     //WORKS: 1.0
    this->pTalonSRX->Config_kP(0, 0.5, 10); //LAST: 0.1
    this->pTalonSRX->Config_kI(0, 0.0, 10); //LAST: 0.0
    this->pTalonSRX->Config_kD(0, 0.0, 10); //LAST: 0.0

    this->pTalonSRX->SetNeutralMode(NeutralMode::Brake);
    this->pVictorSPX->SetNeutralMode(NeutralMode::Brake);
}

/**
 * For when people are yelling at you that code is the problem
 * Sets the track to a PWM output
 * VoltageRamp overrides this
 * Handles like a drunk dolphin without voltageramp
 */
void AdvancedDrive::SetPWM(double power) {
    this->pTalonSRX->Set(ControlMode::PercentOutput, power);
}

/**
 * Does not work, angers the robot, do not use
 */
void AdvancedDrive::SetCurrent(double power) {
    this->pTalonSRX->Set(ControlMode::Current, power);
}

/**
 * Lets us say, "That's not a drivetrain, THIS is a drivetrain"
 * Uses velocity control with linear ramping
 * Corrects all units to SI units
 * Handles like a Falcon hunting
 */
void AdvancedDrive::VelocityTank(double joyX, double joyY) {
    //Speed (in meters per second) drivers want as maximums
    const double targetXVelocity = 1;
    const double targetYVelocity = 3;

    //Meters per Second ^2 at which the bot accelerates
    const double rampAcceleration = 2;

    //Ratio between encoder ticks and two radians (TODO: SI units)
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

    //Simple, linear ramp
    //TODO: Rapid-Decay ramp, "brake" mode
    double deltaYVel = inputYVel - this->currentYVel;
    if ((deltaYVel) > rampCorrectedAcceleration) {
        this->currentYVel += rampCorrectedAcceleration;
    } else if (deltaYVel < -rampCorrectedAcceleration) {
        this->currentYVel += -rampCorrectedAcceleration;
    } else {
        this->currentYVel = inputYVel;
    }

    //Allows same code to be used on either side of the robot
    if (this->mReverseYVel == true) {
        this->pTalonSRX->Set(ControlMode::Velocity, (-this->currentYVel + inputXVel) * encoderPulsesPerRevolution);
    } else {
        this->pTalonSRX->Set(ControlMode::Velocity, (this->currentYVel + inputXVel) * encoderPulsesPerRevolution);
    }
    
}

/**
 * Makes the fancy velocity control work (also needed for motionmagic)
 */
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
    this->pTalonSRX->Config_kI(0, 0.0, 10); //LAST: 0.0
    this->pTalonSRX->Config_kD(0, 6.0, 10); //LAST: 0.0

    //Snappy Robit
    this->pTalonSRX->Set(NeutralMode::Brake);
    //this->pVictorSPX->Set(NeutralMode::Brake);
}

/**
 * Almost a backend call, sets a target velocity for the track to hold
 * Will work when mapped to a joystick (note: accepted unit is ticks-per-second) but
 * If not ramped, leads to a VIOLENT robot
 * Handles like a jumpy raccoon
 */
void AdvancedDrive::SetTargetVelocity(double targetVel) {
    this->pTalonSRX->Set(ControlMode::Velocity, targetVel);
}

/**
 * Sets a target (in ticks) for MotionMagic to servo to
 * Handles like a Greyhound chained to a pole
 */
void AdvancedDrive::SetTargetMotionProfileTarget(double target) {
    this->pTalonSRX->Set(ControlMode::MotionMagic, target);
}

/**
 * Run at init on the left track (or right, depending on how the robot is built)
 */
void AdvancedDrive::SetYVelocityInvert(bool invertState) {
    this->mReverseYVel = invertState;
}