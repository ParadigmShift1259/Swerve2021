/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/SwerveModule.h"


SwerveModule::SwerveModule(int driveMotorChannel, 
                           int turningMotorChannel,
                           GetPulseWidthCallback pulseWidthCallback,
                           CANifier::PWMChannel pwmChannel,
                           bool driveMotorReversed,
                           double offset,
                           const std::string& name)
    : m_offset(offset)
    , m_name(name)
    , m_driveMotor(driveMotorChannel)
    , m_turningMotor(turningMotorChannel, CANSparkMax::MotorType::kBrushless)
    , m_drivePIDLoader("SM", kDriveAdjust, kDriveP, kDriveI, kDriveD, kDriveFF)
    , m_turnPIDLoader("SM", kTurnAdjust, kTurnP, kTurnI, kTurnD, kTurnIZ, kTurnIA)
    , m_pulseWidthCallback(pulseWidthCallback)
    , m_pwmChannel(pwmChannel)
{
    StatorCurrentLimitConfiguration statorLimit { true, kMotorCurrentLimit, kMotorCurrentLimit, 2 };
    m_driveMotor.ConfigStatorCurrentLimit(statorLimit);
    SupplyCurrentLimitConfiguration supplyLimit { true, kMotorCurrentLimit, kMotorCurrentLimit, 2 };
    m_driveMotor.ConfigSupplyCurrentLimit(supplyLimit);
    m_turningMotor.SetSmartCurrentLimit(kMotorCurrentLimit);

    // Set up GetVelocity() to return meters per sec instead of RPM
    m_turnRelativeEncoder.SetPositionConversionFactor(2.0 * wpi::math::pi / kTurnMotorRevsPerWheelRev);
    
    m_driveMotor.SetInverted(driveMotorReversed ? TalonFXInvertType::CounterClockwise : TalonFXInvertType::Clockwise);
    m_turningMotor.SetInverted(false);
    m_driveMotor.ConfigSelectedFeedbackSensor(TalonFXFeedbackDevice::IntegratedSensor);
    m_turnPIDController.SetFeedbackDevice(m_turnRelativeEncoder);
    
    m_turningMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_driveMotor.SetNeutralMode(NeutralMode::Brake);

    m_drivePIDLoader.Load(m_driveMotor);
    m_turnPIDLoader.Load(m_turnPIDController);

    m_timer.Reset();
    m_timer.Start();
}

void SwerveModule::Periodic()
{
    if (m_timer.Get() < 5)
        ResetRelativeToAbsolute();

    double absAngle = CalcAbsoluteAngle();
    SmartDashboard::PutNumber("D_SM_Rel " + m_name, m_turnRelativeEncoder.GetPosition());
    SmartDashboard::PutNumber("D_SM_Abs " + m_name, absAngle);
    SmartDashboard::PutNumber("D_SM_AbsDiff " + m_name, m_turnRelativeEncoder.GetPosition() - absAngle);
    // SmartDashboard::PutNumber("D_SM_MPS " + m_name, CalcMetersPerSec().to<double>());
    // SmartDashboard::PutNumber("D_SM_IError " + m_name, m_turnPIDController.GetIAccum());
    // SmartDashboard::PutNumber("D_SM_TP100MS " + m_name, m_driveMotor.GetSelectedSensorVelocity());
}

SwerveModuleState SwerveModule::GetState()
{
    /// Why do we find the absolute angle here instead of the relative angle?
    return { CalcMetersPerSec(), Rotation2d(radian_t(CalcAbsoluteAngle()))};
}

void SwerveModule::SetDesiredState(SwerveModuleState &state)
{
    // Retrieving PID values from SmartDashboard if enabled
    m_drivePIDLoader.LoadFromNetworkTable(m_driveMotor);
    m_turnPIDLoader.LoadFromNetworkTable(m_turnPIDController);

    // Get relative encoder position
    double currentPosition = m_turnRelativeEncoder.GetPosition();

    // Calculate new turn position given current and desired position
    bool bOutputReverse = false;
    double minTurnRads = MinTurnRads(currentPosition, state.angle.Radians().to<double>(), bOutputReverse);
    double direction = 1.0; // Sent to TalonFX
    if (bOutputReverse)
        direction = -1.0;

    // Set position reference of turnPIDController
    double newPosition = currentPosition + minTurnRads;

    if (state.speed != 0_mps) {
        #ifdef DISABLE_DRIVE
        m_driveMotor.Set(TalonFXControlMode::Velocity, 0.0);
        #else
        m_driveMotor.Set(TalonFXControlMode::Velocity, direction * CalcTicksPer100Ms(state.speed));
        #endif
    }
    else
        m_driveMotor.Set(TalonFXControlMode::PercentOutput, 0.0);

    // Set the angle unless module is coming to a full stop
    if (state.speed.to<double>() != 0.0)
        m_turnPIDController.SetReference(newPosition, ControlType::kPosition);

    // SmartDashboard::PutNumber("D_SM_SetpointMPS " + m_name, state.speed.to<double>());
    // SmartDashboard::PutNumber("D_SM_Error " + m_name, newPosition - m_turnRelativeEncoder.GetPosition());
}

void SwerveModule::ResetEncoders()
{
    m_driveMotor.SetSelectedSensorPosition(0.0);
}

double SwerveModule::CalcAbsoluteAngle()
{
    double pulseWidth = m_pulseWidthCallback(m_pwmChannel);
    // Pulse Width per rotation is not equal for all encoders. Some are 0 - 3865, some are 0 - 4096
    return fmod((pulseWidth - m_offset) * DriveConstants::kPulseWidthToRadians + 2.0 * wpi::math::pi, 2.0 * wpi::math::pi);
    // SmartDashboard::PutNumber("D_SM_PW " + m_name, pulseWidth);
    // SmartDashboard::PutNumber("D_SM_AA " + m_name, absAngle);
    // Convert CW to CCW? absAngle = 2.0 * wpi::math::pi - absAngle;
}

void SwerveModule::ResetRelativeToAbsolute()
{
    m_turnRelativeEncoder.SetPosition(CalcAbsoluteAngle());
}

// Determine the smallest magnitude delta angle that can be added to initial angle that will 
// result in an angle equivalent (but not necessarily equal) to final angle. 
// All angles in radians
// 
// init final   angle1   angle2 
double SwerveModule::MinTurnRads(double init, double final, bool& bOutputReverse)
{
    init = Util::ZeroTo2PiRads(init);
    final = Util::ZeroTo2PiRads(final);

    // The shortest turn angle may be acheived by reversing the motor output direction
    double angle1 = final - init;
    double angle2 = final + wpi::math::pi - init;

    angle1 = Util::NegPiToPiRads(angle1);
    angle2 = Util::NegPiToPiRads(angle2);

    // Choose the smallest angle and determine reverse flag
    //TODO: FINISHED ROBOT TUNING
    // Eventually prefer angle 1 always during high speed to prevent 180s
    // if (fabs(angle1) <= 2 * fabs(angle2))
    // {
        bOutputReverse = false;

        return angle1;
    // } 
    // else
    // {
    //     bOutputReverse = true;

    //     return angle2;
    // }
}

meters_per_second_t SwerveModule::CalcMetersPerSec()
{
   double ticksPer100ms = m_driveMotor.GetSelectedSensorVelocity();
   return meters_per_second_t(kDriveEncoderMetersPerSec * ticksPer100ms);
}

double SwerveModule::CalcTicksPer100Ms(meters_per_second_t speed)
{
   return speed.to<double>() / kDriveEncoderMetersPerSec;
}