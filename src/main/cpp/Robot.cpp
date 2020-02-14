/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //Initialize Objects
  //ToDo: Make this a map file
  this->m_pPrimaryController = new ControllerInput(0);

  this->m_pLeftTrack = new AdvancedDrive(2, 1);
  this->m_pRightTrack = new AdvancedDrive(8, 4);

  //Enable ramped power control
//  this->m_pLeftTrack->InitSimpleRampedControl();
//  this->m_pRightTrack->InitSimpleRampedControl();

  //Prepares current-control
  //DO NOT USE
//  this->m_pLeftTrack->InitCurrentControl();
//  this->m_pRightTrack->InitCurrentControl();

  //Init PIDs for Drivetrain
  this->m_pLeftTrack->InitVelocityControl();
  this->m_pRightTrack->InitVelocityControl();
  this->m_pLeftTrack->VelocityTank(0,0);
  this->m_pRightTrack->VelocityTank(0,0);

  //Set left track to invert Y
  this->m_pLeftTrack->SetYVelocityInvert(true);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

  /**
   * This line will take raw input from the joystick and pass it to the motor controllers;
   * Kept here as LAST RESORT in case motion control breaks
   * m_pTankDrive.setTankDrivePower((0.75 * m_pPrimaryController->getJoyY()), (m_pPrimaryController->getJoyX() * 0.45));
   */

  this->m_pLeftTrack->VelocityTank(this->m_pPrimaryController->getJoyX(), this->m_pPrimaryController->getJoyY());
  this->m_pRightTrack->VelocityTank(this->m_pPrimaryController->getJoyX(), this->m_pPrimaryController->getJoyY());


  //Drive Ratio
//  double lPower = 0.0;
//  double rPower = 0.0;

  //This ratio works for the drivers (NOTE: robot direction is inverted because it moves 'nicer' with the battery in the back)
//  double forwardSpeed = m_pPrimaryController->getJoyY() * 0.75;
//  double turnSpeed = m_pPrimaryController->getJoyX() * 0.35;

  //Basic tank-drive algorithm
//  rPower = (forwardSpeed + turnSpeed);
//  lPower = (-forwardSpeed + turnSpeed);

  //Good old fasioned Tank Drive
//  this->m_pLeftTrack->SetPWM(lPower);
//  this->m_pRightTrack->SetPWM(rPower);

  //Experiment with Rumble
//  if (m_pPrimaryController->getJoyX() > 0) {
//    this->m_pPrimaryController->setRightRumble(m_pPrimaryController->getJoyX());
//  } else if (m_pPrimaryController->getJoyX() < 0) {
//    m_pPrimaryController->setLeftRumble(m_pPrimaryController->getJoyX() * -1);
//  }

  //Begin Crude Vision Playground
  //frc::SmartDashboard::PutNumber("DB/String 0",  visionTable->GetNumber("visionTargetXPos", -1) );
  //frc::SmartDashboard::PutString("DB/String 0", visionTable->GetString("visionTargetXPos", "-1"));
  double sliderPos = -1;
  double visionTargetXPos = frc::SmartDashboard::GetNumber("visionTargetXPos", -1);
  if (visionTargetXPos == -1) {
    sliderPos = -1;
  } else 
  {
    sliderPos = (visionTargetXPos - 320) / 128;
  }
  
  
  frc::SmartDashboard::PutNumber("DB/Slider 0", sliderPos);

  //If testing encoders, use motion profiles to servo to position
//  this->m_pRightTrack->SetTargetMotionProfileTarget(rPower * 4096 * 5);
//  this->m_pLeftTrack->SetTargetMotionProfileTarget(lPower * 4096 * 5);

}

void Robot::TestPeriodic() {

  double sliderPos = -1;
  double visionTargetXPos = frc::SmartDashboard::GetNumber("visionTargetXPos", -1);
  double visionTargetXRatio = visionTargetXPos / (320) - 1;
  if (visionTargetXPos == -1) {
    sliderPos = -1;
  } else 
  {
    sliderPos = (visionTargetXPos - 320) / 128;
  }

  frc::SmartDashboard::PutNumber("DB/Slider 0", visionTargetXRatio);

  this->m_pLeftTrack->VelocityTank(visionTargetXRatio * 0.8, 0);
  this->m_pRightTrack->VelocityTank(visionTargetXRatio * 0.8, 0);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
