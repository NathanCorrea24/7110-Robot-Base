#include "utils/swerve/NeoSwerveModule.h"

#include <numbers>
#include <math.h>
#include <units/voltage.h>

NeoSwerveModule::NeoSwerveModule(SwerveConfig config) :
  m_driveMotor(config.DriveMotorID, rev::CANSparkMax::MotorType::kBrushless),
  m_driveEncoder(m_driveMotor.GetEncoder()),
  m_drivePID(m_driveMotor.GetPIDController()),
  m_turnMotor(config.TurnMotorID, rev::CANSparkMax::MotorType::kBrushless),
  m_turnPID(config.TurnP, config.TurnI, config.TurnD),
  m_turnEncoder(config.TurnEncoderID, config.TurnOffset)
{
  m_driveMotor.RestoreFactoryDefaults();
  m_turnMotor.RestoreFactoryDefaults();

  m_driveMotor.SetIdleMode(config.DriveIdle);
  m_driveMotor.SetSmartCurrentLimit(config.DriveCurrentLimit);
  m_drivePID.SetP(config.DriveP);
  m_drivePID.SetI(config.DriveI);
  m_drivePID.SetD(config.DriveD);
  m_drivePID.SetFF(config.DriveFF);
  m_drivePID.SetOutputRange(-1, 1);
  m_driveEncoder.SetVelocityConversionFactor(config.DriveVelocityRatio);
  m_driveEncoder.SetPositionConversionFactor(config.DrivePositionRatio);

  m_turnMotor.SetIdleMode(config.TurnIdle);
  m_turnMotor.SetSmartCurrentLimit(config.TurnCurrentLimit);
  m_turnPID.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);

  m_driveMotor.BurnFlash();
  m_turnMotor.BurnFlash();

}

frc::SwerveModuleState NeoSwerveModule::GetState()  {
  return {units::meters_per_second_t(m_driveEncoder.GetVelocity()), units::radian_t(m_turnEncoder.Get())};
}

frc::SwerveModulePosition NeoSwerveModule::GetPosition() {
  return {units::meter_t(m_driveEncoder.GetPosition()), units::radian_t(m_turnEncoder.Get())};
}

void NeoSwerveModule::StopMotors() {
  m_driveMotor.Set(0);
  m_turnMotor.Set(0);
}

void NeoSwerveModule::SetDesiredState(const frc::SwerveModuleState& refstate) {
  const auto state = frc::SwerveModuleState::Optimize(
      refstate, units::radian_t(m_turnEncoder.Get()));

  double Turn = m_turnPID.Calculate(m_turnEncoder.Get(), state.angle.Radians().value());

  if (fabs(state.speed.value()) < 0.001) {
    StopMotors();
  } else {
    m_drivePID.SetReference(state.speed.value(), rev::CANSparkMax::ControlType::kVelocity);
    m_turnMotor.Set(Turn);
  }
}

void NeoSwerveModule::ResetEncoders() {
  m_driveEncoder.SetPosition(0);
}

units::celsius_t NeoSwerveModule::GetDriveTemp() {
  return units::celsius_t(m_driveMotor.GetMotorTemperature());
}

units::celsius_t NeoSwerveModule::GetTurnTemp() {
  return units::celsius_t(m_turnMotor.GetMotorTemperature());
}
