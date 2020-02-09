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

    this->pTalonSRX->SetNeutralMode(NeutralMode::Brake);
    this->pVictorSPX->SetNeutralMode(NeutralMode::Brake);
}


void AdvancedDrive::SetPWM(double power) {
    this->pTalonSRX->Set(ControlMode::PercentOutput, power);
}

void AdvancedDrive::SetCurrent(double power) {
    this->pTalonSRX->Set(ControlMode::Current, power);
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
    this->pTalonSRX->Config_kF(0, 0.3, 10);
    this->pTalonSRX->Config_kP(0, 0.8, 10); //LAST: 0.4
    this->pTalonSRX->Config_kI(0, 0.0, 10); //LAST: 0.0
    this->pTalonSRX->Config_kD(0, 6.0, 10); //LAST: 0.0

    //Set Accel and Cruise Velocity
    this->pTalonSRX->ConfigMotionCruiseVelocity(1000, 10);
    this->pTalonSRX->ConfigMotionAcceleration(800, 10);

    this->pTalonSRX->ConfigVoltageCompSaturation(10, 10);
    this->pVictorSPX->ConfigVoltageCompSaturation(10, 10);

    //Snappy Robit
    this->pTalonSRX->Set(NeutralMode::Brake);
    //this->pVictorSPX->Set(NeutralMode::Brake);
}

void AdvancedDrive::SetTargetVelocity(double targetVel) {
    this->pTalonSRX->Set(ControlMode::Velocity, targetVel);
}

void AdvancedDrive::SetTargetMotionProfileTarget(double target) {
    this->pTalonSRX->Set(ControlMode::MotionMagic, target);
}