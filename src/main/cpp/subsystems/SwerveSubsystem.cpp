#include "subsystems/SwerveSubsystem.h"

#include "SwerveConfigs.h"
#include "Constants.h"
#include "utils/Math.h"
#include <wpi/sendable/SendableBuilder.h>

#define LAMBDA(x) [this] {return x;}

using namespace hb;

SwerveSubsystem::SwerveSubsystem() :
  m_FrontLeft(SwerveConfigs::GetInstance().GetFrontLeft()),
  m_FrontRight(SwerveConfigs::GetInstance().GetFrontRight()), 
  m_RearLeft(SwerveConfigs::GetInstance().GetRearLeft()),
  m_RearRight(SwerveConfigs::GetInstance().GetRearRight()), 
  m_Gyro(0), 
  m_ModulePositions(
    m_FrontLeft.GetPosition(),
    m_FrontRight.GetPosition(),
    m_RearLeft.GetPosition(),
    m_RearRight.GetPosition()
  ), 
  m_Odometry(m_Kinematics, m_Gyro.GetRot2d(),
    m_ModulePositions, frc::Pose2d()),
  m_PoseEstimation(m_Kinematics, m_Gyro.GetRot2d(), 
    m_ModulePositions, frc::Pose2d()) {}

void SwerveSubsystem::Periodic() {
  
  m_ModulePositions[0] = m_FrontLeft.GetPosition();
  m_ModulePositions[1] = m_FrontRight.GetPosition();
  m_ModulePositions[2] = m_RearLeft.GetPosition();
  m_ModulePositions[3] = m_RearRight.GetPosition();

  m_Odometry.Update(m_Gyro.GetRot2d(), m_ModulePositions);

  m_PoseEstimation.Update(m_Gyro.GetRot2d(), m_ModulePositions);
}

void SwerveSubsystem::Drive(units::meters_per_second_t x, 
                            units::meters_per_second_t y,
                            units::radians_per_second_t rot, 
                            bool field) {
  
  auto states = m_Kinematics.ToSwerveModuleStates(
    field ? 
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      x, y, rot, m_Gyro.GetRot2d()) 
      : 
      frc::ChassisSpeeds(x, y, rot)
  );

  m_Kinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, rl, rr] = states;

  m_FrontLeft.SetDesiredState(fl);
  m_FrontRight.SetDesiredState(fr);
  m_RearLeft.SetDesiredState(rl);
  m_RearRight.SetDesiredState(rr);
}

void SwerveSubsystem::Drive(units::meters_per_second_t x,
                            units::meters_per_second_t y,
                            units::radians_per_second_t rot,
                            bool field,
                            bool scaleInputs) {

  if (scaleInputs) {
    x = units::meters_per_second_t(sgn(x.value()) * pow(x.value(), 2));
    y = units::meters_per_second_t(sgn(y.value()) * pow(y.value(), 2));
  }

  auto states = m_Kinematics.ToSwerveModuleStates(
    field ? 
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      x, y, rot, m_Gyro.GetRot2d()) 
      : 
      frc::ChassisSpeeds(x, y, rot)
  );

  m_Kinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, rl, rr] = states;

  m_FrontLeft.SetDesiredState(fl);
  m_FrontRight.SetDesiredState(fr);
  m_RearLeft.SetDesiredState(rl);
  m_RearRight.SetDesiredState(rr);  
  
}

void SwerveSubsystem::ZeroHeading(units::degree_t heading = 0_deg) {
  m_Gyro.SetPosition(heading);
} 

frc::Pose2d SwerveSubsystem::GetPose() const {
  return m_PoseEstimation.GetEstimatedPosition();
}

void SwerveSubsystem::ResetOdometry(const frc::Pose2d& pose) {
  m_Odometry.ResetPosition(m_Gyro.GetRot2d(), m_ModulePositions, pose);
}

// void SwerveSubsystem::InitSendable(wpi::SendableBuilder& builder) {
//   builder.SetSmartDashboardType("Swerve Subsystem");

//   builder.AddDoubleProperty("FLD Temp", LAMBDA(m_FrontLeft.GetDriveTemp().value()), nullptr);
//   builder.AddDoubleProperty("FLT Temp", LAMBDA(m_FrontLeft.GetTurnTemp().value()), nullptr);
//   builder.AddDoubleProperty("FL Azimuth", LAMBDA(m_FrontLeft.GetState().angle.Radians().value()), nullptr);
//   builder.AddDoubleProperty("FL Speed", LAMBDA(m_FrontLeft.GetState().speed.value()), nullptr);

//   builder.AddDoubleProperty("FRD Temp", LAMBDA(m_FrontRight.GetDriveTemp().value()), nullptr);
//   builder.AddDoubleProperty("FRT Temp", LAMBDA(m_FrontRight.GetTurnTemp().value()), nullptr);
//   builder.AddDoubleProperty("FR Azimuth", LAMBDA(m_FrontRight.GetState().angle.Radians().value()), nullptr);
//   builder.AddDoubleProperty("FR Speed", LAMBDA(m_FrontRight.GetState().speed.value()), nullptr);

//   builder.AddDoubleProperty("RLD Temp", LAMBDA(m_RearLeft.GetDriveTemp().value()), nullptr);
//   builder.AddDoubleProperty("RLT Temp", LAMBDA(m_RearLeft.GetTurnTemp().value()), nullptr);
//   builder.AddDoubleProperty("RL Azimuth", LAMBDA(m_RearLeft.GetState().angle.Radians().value()), nullptr);
//   builder.AddDoubleProperty("RL Speed", LAMBDA(m_RearLeft.GetState().speed.value()), nullptr);

//   builder.AddDoubleProperty("RRD Temp", LAMBDA(m_RearRight.GetDriveTemp().value()), nullptr);
//   builder.AddDoubleProperty("RRT Temp", LAMBDA(m_RearRight.GetTurnTemp().value()), nullptr);
//   builder.AddDoubleProperty("RR Azimuth", LAMBDA(m_RearRight.GetState().angle.Radians().value()), nullptr);
//   builder.AddDoubleProperty("RR Speed", LAMBDA(m_RearRight.GetState().speed.value()), nullptr);

//   builder.AddDoubleProperty("Heading", LAMBDA(m_Gyro.GetRot2d().Degrees().value()), nullptr);

// }
