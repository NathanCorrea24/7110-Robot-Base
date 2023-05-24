// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/PowerDistribution.h>
#include <frc2/command/button/CommandXboxController.h>


#include "Constants.h"

//Subsystems
#include "subsystems/SwerveSubsystem.h"

//Commands

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  // The driver's controller
  frc::XboxController m_driverController{0};
  frc::XboxController m_operatorController{1};
  frc2::CommandXboxController m_driver{2};

  // The robot's subsystems and commands are defined here...

  frc::SlewRateLimiter<units::scalar> m_speedLimitx{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_speedLimity{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_speedLimitz{3 / 1_s};

  // The robot's subsystems
  SwerveSubsystem m_drive;

  // The robot's commands

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
};