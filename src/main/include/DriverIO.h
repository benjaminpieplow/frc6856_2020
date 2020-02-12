#pragma once

#include <frc/Joystick.h>
//#include <frc/WPILib.h>
//#include <cstdlib>

/**
 * A basic xBox controller and calls to get data from it
 */
class ControllerInput
{
    public:
    ControllerInput(int controllerID);
    ~ControllerInput();

    //Returns Joystick X Axis
    double getJoyX();

    //Returns Joystick Y Axis
    double getJoyY();

    //Sets Left Rumble
    void setLeftRumble(double power);

    //Sets Right Rumble
    void setRightRumble(double power);


    private:
    //Operator's Joystick
    frc::Joystick* m_pController;

};