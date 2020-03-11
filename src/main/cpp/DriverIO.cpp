#include <DriverIO.h>

/**
 * Code for an X-Box style controller
 */

/**
 * Creates the controller using the supplied number
 */
ControllerInput::ControllerInput(int controllerID) {
    //Create a joystick on specified port
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

bool ControllerInput::getRawButton(int button) {
    return this->m_pController->GetRawButton(button);
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

double ControllerInput::getLTrigger() {
    return this->m_pController->GetRawAxis(2);
}


/**
 * Code for a flight joystick
 */


JoystickInput::JoystickInput(int controllerID)
{
    //Create a joystick on specified port
    this->m_pJoystick = new frc::Joystick(controllerID);
}

JoystickInput::~JoystickInput()
{
}

/**
 * Returns the Joystick X axis
 * Currently Used for testing Power Control
 */
double JoystickInput::getJoyX()
{
    return this->m_pJoystick->GetRawAxis(0);
}

/**
 * Returns Joystick Y Axis
*/
double JoystickInput::getJoyY()
{
    return this->m_pJoystick->GetRawAxis(1);
}

bool JoystickInput::getRawButton(int button) {
    return this->m_pJoystick->GetRawButton(button);
}
