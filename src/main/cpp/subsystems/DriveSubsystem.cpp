// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <math.h>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

#define LAMBDA(x) [this] {return x;}

using namespace DriveConstants;
using namespace DriveConstants::CanIds;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDriveMotorPort,
                  kFrontLeftTurningMotorPort,
                  kFrontLeftTurningEncoderPorts,
                  kFrontLeftOffset},

      m_rearLeft{
          kRearLeftDriveMotorPort,       kRearLeftTurningMotorPort,
          kRearLeftTurningEncoderPorts,  kRearLeftOffset},

      m_frontRight{
          kFrontRightDriveMotorPort,       kFrontRightTurningMotorPort,
          kFrontRightTurningEncoderPorts,  kFrontRightOffset},

      m_rearRight{
          kRearRightDriveMotorPort,       kRearRightTurningMotorPort,
          kRearRightTurningEncoderPorts,  kRearRightOffset},

      m_speed(DriveConstants::kMaxSpeed.value()), 

      m_odometry(kDriveKinematics, gyro.GetRot2d(), {m_frontLeft.GetPosition(),
                    m_rearLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_rearRight.GetPosition()}, frc::Pose2d()) {SetName("Swerve"); SetTelemetry();}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(gyro.GetRot2d(), {m_frontLeft.GetPosition(),
                    m_rearLeft.GetPosition(), m_frontRight.GetPosition(),
                    m_rearRight.GetPosition()});

}

std::unordered_map<std::string, std::function<double()>> DriveSubsystem::GetTelemetry() {
  return m_telemetry;
}

void DriveSubsystem::SetTelemetry() {
  // Front Left Telemetry
  m_telemetry.insert({"FL Speed" , LAMBDA(m_frontLeft.GetState().speed.value())});
  m_telemetry.insert({"FL Angle" , LAMBDA(m_frontLeft.GetState().angle.Degrees().value())});
  m_telemetry.insert({"FL D Temp", LAMBDA(m_frontLeft.GetDriveMotorTemp().value())});
  m_telemetry.insert({"FL T Temp", LAMBDA(m_frontLeft.GetTurnMotorTemp().value())});


  // Front Right Telemetry
  m_telemetry.insert({"FR Speed" , LAMBDA(m_frontRight.GetState().speed.value())});
  m_telemetry.insert({"FR Angle" , LAMBDA(m_frontRight.GetState().angle.Degrees().value())});
  m_telemetry.insert({"FR D Temp", LAMBDA(m_frontRight.GetDriveMotorTemp().value())});
  m_telemetry.insert({"FR T Temp", LAMBDA(m_frontRight.GetTurnMotorTemp().value())});

  // Rear Left Telemetry
  m_telemetry.insert({"RL Speed" , LAMBDA(m_rearLeft.GetState().speed.value())});
  m_telemetry.insert({"RL Angle" , LAMBDA(m_rearLeft.GetState().angle.Degrees().value())});
  m_telemetry.insert({"RL D Temp", LAMBDA(m_rearLeft.GetDriveMotorTemp().value())});
  m_telemetry.insert({"RL T Temp", LAMBDA(m_rearLeft.GetTurnMotorTemp().value())});

  // Rear Right Telemetry
  m_telemetry.insert({"RR Speed" , LAMBDA(m_rearRight.GetState().speed.value())});
  m_telemetry.insert({"RR Angle" , LAMBDA(m_rearRight.GetState().angle.Degrees().value())});
  m_telemetry.insert({"RR D Temp", LAMBDA(m_rearRight.GetDriveMotorTemp().value())});
  m_telemetry.insert({"RR T Temp", LAMBDA(m_rearRight.GetTurnMotorTemp().value())});

  m_telemetry.insert({"Gyro Angle", LAMBDA(gyro.GetRot2d().Degrees().value())});

  m_telemetry.insert({"Max Speed", LAMBDA(GetSpeed().value())});
}

hb::SubsystemData DriveSubsystem::GetData() {
  return {GetName(), GetTelemetry()};
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, gyro.GetRot2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radian_t heading) {
    units::radians_per_second_t rot = units::radians_per_second_t(m_turnController.Calculate(gyro.GetRad(), heading));

    auto states = kDriveKinematics.ToSwerveModuleStates(
    frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      xSpeed, ySpeed, rot, gyro.GetRotation2d()
    ));

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);
  desiredStates[0].angle = ((desiredStates[0].angle * -1.0));
  desiredStates[1].angle = ((desiredStates[1].angle * -1.0));
  desiredStates[2].angle = ((desiredStates[2].angle * -1.0));
  desiredStates[3].angle = ((desiredStates[3].angle * -1.0));
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ZeroHeading() {
  gyro.Reset();
}

double DriveSubsystem::GetTurnRate() {
  return -gyro.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(frc::Rotation2d(units::degree_t(gyro.GetAngle())), 
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                    m_frontRight.GetPosition(), m_rearRight.GetPosition()},
                    pose);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_rearRight.ResetEncoders();
}