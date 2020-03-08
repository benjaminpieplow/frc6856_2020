/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * Lessons Learned:
 * 1) Opening curly brace goes on the line BELOW, comments go to the RIGHT, as you write!
 * 2) Write code from-the-ground-up to match Mechanical.
 *  They'll want to test systems, create each system (motor/s) as a class,
 *  make the devices private members
 *  then write accessors for the motors (or whatever). This will scale MUCH nicer
 */


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
  this->m_pSecondaryController = new ControllerInput(1);

  this->m_pLeftTrack = new AdvancedDrive(10, 11, true);
  this->m_pRightTrack = new AdvancedDrive(12, 13, false);

  //Shooter Falcon
  this->m_pTestShooter = new Shooter(40);

  //Turret System
  this->m_pTestTurret = new Turret(16);

  //Elevintake System
  this->m_pElevator = new Elevator(26);

  //Ball Feeder System
  this->m_pFeeder = new Feeder(17);

  //Master Ball System
  this->m_pBallSystem = new BallSystem(this->m_pElevator, this->m_pTestShooter, this->m_pTestTurret, this->m_pIntakeArm);

  //Intake System
  this->m_pIntakeArm = new IntakeArm(24);

  //Climber
  this->m_pLiftSystem = new LiftSystem(27);

  //Autonomous system - controls robot during Autonomous
  this->m_pAuton = new Autonomous(this->m_pTestTurret, this->m_pTestShooter, this->m_pLeftTrack, this->m_pRightTrack, this->m_pElevator, m_pFeeder);

  //Init PIDs for Drivetrain
  this->m_pLeftTrack->InitVelocityControl();
  this->m_pRightTrack->InitVelocityControl();
  //Zero the drivetrain
  this->m_pLeftTrack->VelocityTank(0,0);
  this->m_pRightTrack->VelocityTank(0,0);

//  this->m_pLeftTrack->SetYVelocityInvert(true);


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
  /**
   * Original Auton
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
      } else {
  }
  */
  this->m_pAuton->CrudeAutonInit();
}

void Robot::AutonomousPeriodic() {
  frc::SmartDashboard::PutBoolean("DB/LED 0", false);
  frc::SmartDashboard::PutBoolean("DB/LED 1", false);
  frc::SmartDashboard::PutBoolean("DB/LED 2", false);
  this->m_pAuton->CrudeAutonPeriodic();
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

  /**
   * This line will take raw input from the joystick and pass it to the motor controllers;
   * Kept here as LAST RESORT in case motion control breaks
   * m_pTankDrive.setTankDrivePower((0.75 * m_pPrimaryController->getJoyY()), (m_pPrimaryController->getJoyX() * 0.45));
   */


  /**
   * VelocityTank will hold the wheels at a velocity, set in meters per second
   * Boost will increase this speed, but reduce steering
   * Currently, either the integral accumulator or (more likely) a combination of Boost and maximum velocity not being reached
   * prevents the robot from decelerating on demand. Fix this when you have a working drive base.
   */

  this->m_pTestShooter->ToggleAutoRPM(this->m_pPrimaryController->getRawButton(6), this->m_pPrimaryController->getRawButton(5));

  //Disable BOOST for new drivers
//      this->m_pLeftTrack->VelocityTank(this->m_pPrimaryController->getJoyX() * -1, this->m_pPrimaryController->getJoyY(), this->m_pPrimaryController->getRTrigger());
//      this->m_pRightTrack->VelocityTank(this->m_pPrimaryController->getJoyX(), this->m_pPrimaryController->getJoyY(), this->m_pPrimaryController->getRTrigger());
    this->m_pLeftTrack->VelocityTank(this->m_pPrimaryController->getJoyX() * -1, this->m_pPrimaryController->getJoyY(), 0);
    this->m_pRightTrack->VelocityTank(this->m_pPrimaryController->getJoyX(), this->m_pPrimaryController->getJoyY(), 0);

  if (this->m_pPrimaryController->getRawButton(5)) {
    this->m_pTestShooter->DisableShooter();
  }

  if (this->m_pPrimaryController->getRawButton(3)) {
    this->m_pFeeder->FeedForward();
    this->m_pElevator->ElevatorForward();
  } else {
    this->m_pFeeder->FeedStop();
    this->m_pElevator->ElevatorStop();
  }


  if (this->m_pPrimaryController->getRTrigger() > 0.25)
  {
    this->m_pElevator->ElevatorForward();
  }
  else if (this->m_pPrimaryController->getLTrigger() > 0.25)
  {
    this->m_pElevator->ElevatorReverse();
  }
  else if (!this->m_pPrimaryController->getRawButton(3))
  {
    this->m_pElevator->ElevatorStop();
  }

  

/**
  if (this->m_pPrimaryController->getRawButton(1)) {
    this->m_pElevator->ElevatorStop();
  } else if (this->m_pPrimaryController->getRawButton(2)) {
    this->m_pElevator->ElevatorReverse();
  } else {
    this->m_pElevator->ElevatorForward();
  }
  */

}


void Robot::TestPeriodic() {


  double currentRPM = m_pTestShooter->GetShooterRPM();
  double newRPM = currentRPM + 10 * this->m_pPrimaryController->getJoyY() * -1;
  frc::SmartDashboard::PutNumber("DB/Slider 1", newRPM / 1000);
  frc::SmartDashboard::PutNumber("DB/Slider 2", this->m_pTestShooter->GetTargetDistance());
  frc::SmartDashboard::PutNumber("DB/Slider 3", this->m_pTestShooter->GetTargetCalculatedRPM());


  this->m_pTestShooter->ToggleAutoRPM(this->m_pPrimaryController->getRawButton(6), this->m_pPrimaryController->getRawButton(5));

    frc::SmartDashboard::PutBoolean("DB/LED 0", false);
    frc::SmartDashboard::PutBoolean("DB/LED 1", false);
    frc::SmartDashboard::PutBoolean("DB/LED 2", false);

  if (this->m_pPrimaryController->getRawButton(3)) {
    this->m_pFeeder->FeedForward();
    this->m_pElevator->ElevatorForward();
  } else {
    this->m_pFeeder->FeedStop();
    this->m_pElevator->ElevatorStop();
  }
//    frc::SmartDashboard::PutBoolean("DB/LED 3", false);

/**
 if (visionTargetXPos < 0) {
   this->m_pTestTurret->SetTurretPower(0);
 } else {
   this->m_pTestTurret->SetTurretPower(visionTargetXRatio);
 }
*/


/*
  if (this->m_pPrimaryController->getRawButton(1)) {
    if (this->m_pTestTurret->AutoTurret()) {
      if (this->m_pTestShooter->AutoRPM()) {
        this->m_pElevator->FeederForward();
      } else {
        this->m_pElevator->ElevatorStop();
      }
    }
  } else {
  }
*/

/**
  if (this->m_pPrimaryController->getRawButton(3)) {
    this->m_pElevator->FeederForward();
  } else {
    this->m_pElevator->FeederStop();
  }
  


  if (this->m_pPrimaryController->getRawButton(5)) {
    this->m_pIntakeSystem->RaiseIntake();
  } else if (this->m_pPrimaryController->getRawButton(6)) {
    this->m_pIntakeSystem->LowerIntake();
  } else
  {
    this->m_pIntakeSystem->IntakePeriodic();
  }

  */

  //this->m_pElevator->SetElevatorPower(this->m_pPrimaryController->getJoyX());
  
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
