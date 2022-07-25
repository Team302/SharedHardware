// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>

#include <auton/CyclePrimitives.h>
#include <TeleopControl.h>
#include <chassis/swerve/SwerveDrive.h>
#include <hw/DragonLimelight.h>
#include <mechanisms/climber/ClimberStateMgr.h>
#include <mechanisms/indexer/IndexerStateMgr.h>
#include <mechanisms/intake/IntakeStateMgr.h>
#include <mechanisms/lift/LiftStateMgr.h>
#include <mechanisms/shooter/ShooterStateMgr.h>
#include <chassis/IChassis.h>



class Robot : public frc::TimedRobot 
{
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  TeleopControl*        m_controller;
  IChassis*             m_chassis;
  CyclePrimitives*      m_cyclePrims;
  frc::Timer*           m_timer;
  SwerveDrive*          m_swerve;

  IntakeStateMgr*       m_leftIntakeStateMgr;
  IntakeStateMgr*       m_rightIntakeStateMgr;
  IndexerStateMgr*      m_indexerStateMgr;
  LiftStateMgr*         m_liftStateMgr;
  ShooterStateMgr*      m_shooterStateMgr;
  ClimberStateMgr*      m_climberStateMgr;
  DragonLimelight*      m_dragonLimeLight;
};
