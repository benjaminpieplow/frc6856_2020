/**
 * All Air Actuators!
 * Stolen verbatim from last year, because we legit killed it with this class:
 * https://github.com/cuthbertnibbles/frc6856_2019/blob/master/src/main/include/Pneumatics.h
 */

#pragma once

#include <frc/DoubleSolenoid.h>
#include <iostream>
#include <frc/Compressor.h>

class PneumaticActuator
{
    public:
    PneumaticActuator(int solenoidForwardPort, int solenoidReversePort);
    ~PneumaticActuator();

    //Switch the Actuator, will only work on the "rising edge" of the trigger pull
    void togglePneumaticActuator(bool trigger);

    //Set the Actuator State
    void setPneumaticActuator(bool forwards);


    private:
    int m_pSolenoidPortOne;
    int m_pSolenoidPortTwo;
    frc::DoubleSolenoid* m_pActuator;

    //Keep track of current trigger changes
    bool hasChanged = true;
};

class ManualCompressor
{
    public:
    ManualCompressor();
    ~ManualCompressor();

    void enableCompressor();

    void disableCompressor();

    void setCompressor(bool trigger);

    void toggleCompressor(bool trigger);

    private:

    frc::Compressor *m_pCompressor;

    bool hasChanged = true;
};