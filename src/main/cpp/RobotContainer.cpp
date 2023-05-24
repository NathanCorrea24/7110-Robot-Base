// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "RobotContainer.h"

#include <utility>
#include <cmath>
#include <numbers>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/ConditionalCommand.h>

#include "Constants.h"
#include "subsystems/SwerveSubsystem.h"
#include "utils/cams/Limelight.h"


using namespace DriveConstants;


RobotContainer::RobotContainer() {

  /**
   * Adding options to chooser should be done as such
   * m_chooser.AddOption("Auto Name", &Auto);
   * Where auto has already been declared in RobotContainer.h
  */

  frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);

  // Configure the button bindings
  ConfigureButtonBindings();

  //Default subsystem commands are defined in this section

}

void RobotContainer::ConfigureButtonBindings() {
  /**
   *  When binding buttons use this as the template
   *  frc::JoystickButton(&m_controller, frc::XboxController::Button::kButton).WhenPressed(Command);
   *  TODO: test command xbox controller implementation(found below) and see if it works
  */
  m_driver.A().WhenActive(frc2::InstantCommand([this] {m_drive.ZeroHeading(0_deg);}));
}


frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_chooser.GetSelected();
}
