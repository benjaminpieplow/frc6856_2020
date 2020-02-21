
#include <DriverIO.h>

/**
 * Creates the controller using the supplied number
 */
ControllerInput::ControllerInput(int controllerID) {
    //Create a joystick on port 1
    this->m_pController = new frc::Joystick(controllerID);
}

/**
 * Returns the Joystick X axis
 * Currently Used for testing Power Control
 */
double ControllerInput::getJoyX()
{
    return this->m_pController->GetRawAxis(4);
}

/**
 * Returns Joystick Y Axis
*/
double ControllerInput::getJoyY()
{
    return this->m_pController->GetRawAxis(1);
}

void ControllerInput::setLeftRumble(double power) {
    this->m_pController->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, power);
}

void ControllerInput::setRightRumble(double power) {
    this->m_pController->SetRumble(frc::GenericHID::RumbleType::kRightRumble, power);
}

double ControllerInput::getRTrigger() {
    return this->m_pController->GetRawAxis(3);
}