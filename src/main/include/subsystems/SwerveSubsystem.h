#pragma once

#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc2/command/SubsystemBase.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include "utils/swerve/NeoSwerveModule.h"
#include "utils/swerve/PigeonGyro.h"
#include "Constants.h"

class SwerveSubsystem : public frc2::SubsystemBase {
  public:

    SwerveSubsystem();

    void Periodic() override;

    void Drive(units::meters_per_second_t x, units::meters_per_second_t y,
               units::radians_per_second_t rot, bool field);

    void Drive(units::meters_per_second_t x, units::meters_per_second_t y,
               units::radians_per_second_t rot, bool field, bool scaleInputs);

    void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states);

    void ZeroHeading(units::degree_t heading);

    frc::Pose2d GetPose() const;

    void ResetOdometry(const frc::Pose2d& pose);

    // void InitSendable(wpi::SendableBuilder& builder) override;

  private:

    NeoSwerveModule m_FrontLeft;
    NeoSwerveModule m_FrontRight;
    NeoSwerveModule m_RearLeft;
    NeoSwerveModule m_RearRight;

    hb::pigeonGyro m_Gyro;

    frc::SwerveDriveKinematics<4> m_Kinematics {
      frc::Translation2d(DriveConstants::kTrackLength, -DriveConstants::kTrackWidth),
      frc::Translation2d(DriveConstants::kTrackLength, DriveConstants::kTrackWidth),
      frc::Translation2d(-DriveConstants::kTrackLength, -DriveConstants::kTrackWidth),
      frc::Translation2d(-DriveConstants::kTrackLength, DriveConstants::kTrackWidth)
    };

    wpi::array<frc::SwerveModulePosition, 4> m_ModulePositions;

    frc::SwerveDriveOdometry<4> m_Odometry;

    frc::SwerveDrivePoseEstimator<4> m_PoseEstimation;
};