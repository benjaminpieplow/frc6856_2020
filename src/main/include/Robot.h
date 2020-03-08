/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <DriveTrain.h>

#include <DriverIO.h>

#include <Elevator.h>
#include <Shooter.h>
#include <Turret.h>
#include <IntakeArm.h>
#include <LiftSystem.h>
#include <Feeder.h>

#include <Autonomous.h>

#include <BallSystem.h>

//Maybe we can move this somewhere else?
#include <ntcore.h>
#include <networktables/NetworkTable.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;


//  TankDrive m_pTankDrive;
  ControllerInput* m_pPrimaryController;
  ControllerInput* m_pSecondaryController;

  AdvancedDrive* m_pLeftTrack;
  AdvancedDrive* m_pRightTrack;

  NetworkTable* visionTable;

  Shooter* m_pTestShooter;

  Turret* m_pTestTurret;

  Elevator* m_pElevator;

  BallSystem* m_pBallSystem;

  IntakeArm* m_pIntakeArm;

  Autonomous* m_pAuton;

  LiftSystem* m_pLiftSystem;

  Feeder* m_pFeeder;
};
