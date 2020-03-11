#pragma once

#include <frc/Joystick.h>
//#include <frc/WPILib.h>
//#include <cstdlib>

class ControllerInput
{
    public:
    ControllerInput(int controllerID);
    ~ControllerInput();

    //Returns Joystick X Axis
    double getJoyX();

    //Returns Joystick Y Axis
    double getJoyY();

    //Returns a bool for a specified button
    bool getRawButton(int button);

    //Returns right trigger
    double getRTrigger();

    //Returns left trigger
    double getLTrigger();



    //Sets Left Rumble
    void setLeftRumble(double power);

    //Sets Right Rumble
    void setRightRumble(double power);


    private:
    //Operator's Joystick
    frc::Joystick* m_pController;

};

class JoystickInput
{
public:
    JoystickInput(int controllerID);
    ~JoystickInput();


    //Returns Joystick X Axis
    double getJoyX();

    //Returns Joystick Y Axis
    double getJoyY();

    //Returns a bool for a specified button
    bool getRawButton(int button);

private:
    frc::Joystick* m_pJoystick;

};

