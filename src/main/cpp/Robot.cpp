/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <cameraserver/CameraServer.h>


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


  //Initialize Objects
  //ToDo: Make this a map file
  this->m_pPrimaryController = new ControllerInput(0);

  this->m_pLeftTrack = new AdvancedDrive(2, 1);
  this->m_pRightTrack = new AdvancedDrive(8, 4);

  //Shooter Falcon
  this->m_pTestShooter = new Shooter(40, 16);

  //Turret System
  this->m_pTestTurret = new Turret(26);

  //Elevintake System
  this->m_pElevator = new Elevator(24, 17);

  //Master Ball System
  this->m_pBallSystem = new BallSystem(this->m_pElevator, this->m_pTestShooter, m_pTestTurret);

  //Intake System
  this->m_pIntakeSystem = new Intake(27);

  //Init PIDs for Drivetrain
  this->m_pLeftTrack->InitVelocityControl();
  this->m_pRightTrack->InitVelocityControl();
  this->m_pLeftTrack->VelocityTank(0,0);
  this->m_pRightTrack->VelocityTank(0,0);

  //Set left track to invert Y
  this->m_pLeftTrack->SetYVelocityInvert(true);


    frc::SmartDashboard::PutBoolean("DB/LED 0", false);
    frc::SmartDashboard::PutBoolean("DB/LED 1", false);
    frc::SmartDashboard::PutBoolean("DB/LED 2", false);
    frc::SmartDashboard::PutBoolean("DB/LED 3", false);
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

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

  /**
   * This line will take raw input from the joystick and pass it to the motor controllers;
   * Kept here as LAST RESORT in case motion control breaks
   * m_pTankDrive.setTankDrivePower((0.75 * m_pPrimaryController->getJoyY()), (m_pPrimaryController->getJoyX() * 0.45));
   */

  this->m_pLeftTrack->VelocityTank(this->m_pPrimaryController->getJoyX(), this->m_pPrimaryController->getJoyY());
  this->m_pRightTrack->VelocityTank(this->m_pPrimaryController->getJoyX(), this->m_pPrimaryController->getJoyY());


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

  double currentRPM = m_pTestShooter->GetShooterRPM();
  double newRPM = currentRPM + 10 * this->m_pPrimaryController->getJoyY() * -1;
  frc::SmartDashboard::PutNumber("DB/Slider 0", newRPM / 1000);

 if (this->m_pPrimaryController->getRawButton(5)) {
//     this->m_pTestShooter->EnableShooter(newRPM);
 }
 if (this->m_pPrimaryController->getRawButton(6)) {
   this->m_pTestShooter->DisableShooter();
 }

/**
  if (this->m_pPrimaryController->getRawButton(1)) {
    this->m_pElevator->ElevatorForward();
  } else {
    this->m_pElevator->ElevatorStop();
  }

  if (this->m_pPrimaryController->getRawButton(2)) {
    this->m_pElevator->FeederForward();
  } else {
    this->m_pElevator->FeederStop();
  }
  */


    frc::SmartDashboard::PutBoolean("DB/LED 0", false);
    frc::SmartDashboard::PutBoolean("DB/LED 1", false);
    frc::SmartDashboard::PutBoolean("DB/LED 2", false);
//    frc::SmartDashboard::PutBoolean("DB/LED 3", false);

/**
 if (visionTargetXPos < 0) {
   this->m_pTestTurret->SetTurretPower(0);
 } else {
   this->m_pTestTurret->SetTurretPower(visionTargetXRatio);
 }
*/

  frc::SmartDashboard::PutNumber("DB/Slider 1", this->m_pTestTurret->GetFOVXAngle());
  frc::SmartDashboard::PutNumber("DB/Slider 2", this->m_pTestTurret->GetTurretFrameAngle());
  frc::SmartDashboard::PutNumber("DB/Slider 3", this->m_pTestTurret->GetTargetFrameAngle());
//  frc::SmartDashboard::PutBoolean("DB/LED 1", this->m_pTestTurret->GetTurretLocked());

  if (this->m_pPrimaryController->getRawButton(1)) {
    this->m_pTestTurret->AutoTurret();

  } else {
//    this->m_pTestTurret->SetTurretAngle(this->m_pPrimaryController->getJoyX() * 20);
    this->m_pTestTurret->SetTurretPower(this->m_pPrimaryController->getJoyX() * 0.3);
  }


  if (this->m_pPrimaryController->getRawButton(5)) {
    this->m_pIntakeSystem->RaiseIntake();
  } else if (this->m_pPrimaryController->getRawButton(6)) {
    this->m_pIntakeSystem->LowerIntake();
  } else
  {
    this->m_pIntakeSystem->IntakePeriodic();
  }
  

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
