/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/AnalogInput.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <networktables/NetworkTableEntry.h>
#include <wpi/math>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"

#include <rev\CANSparkMax.h>

#pragma GCC diagnostic pop

#include <string>

#include "Constants.h"
#include "common/Util.h"

#include "common/PIDLoaderNEO.h"
 
// Uncomment this to prevent swerve modules from driving
//#define DISABLE_DRIVE

using namespace rev;
using namespace ModuleConstants;
using namespace units;
using namespace std;
using namespace frc;

using GetPulseWidthCallback = function<double (CANifier::PWMChannel)>;



class SwerveModule
{
    using radians_per_second_squared_t = compound_unit<radians, inverse<squared<second>>>;

public:
    SwerveModule( int driveMotorChannel
                , int turningMotorChannel
                , const int turningEncoderPort
                , bool driveEncoderReversed
                , double offSet
                , const std::string& name);

    void Periodic();

    /// Get the state for the swerve module pod
    /// \return             The state (vector with speed and angle) representig the current module state
    /// @todo Currently GetState uses the absolute angle instead of the relative angle that we should be using
    frc::SwerveModuleState GetState();

    /// Set the desired state for the swerve module pod
    /// \param state        The state (vector with speed and angle) representing the desired module state
    void SetDesiredState(frc::SwerveModuleState &state);

    /// Resets the drive motor encoders to 0
    void ResetEncoders();
    /// Resync the relative NEO turn encoder to the absolute encoder
    void ResetRelativeToAbsolute();

    // Convert any angle theta in radians to its equivalent on the interval [0, 2pi]
    static double ZeroTo2PiRads(double theta);

    // Convert any angle theta in radians to its equivalent on the interval [-pi, pi]
    static double NegPiToPiRads(double theta);

private:
    double VoltageToRadians(double voltage, double Offset);
    double VoltageToDegrees(double voltage, double Offset);

    // Determine the smallest magnitude delta angle that can be added to initial angle that will 
    // result in an angle equivalent (but not necessarily equal) to final angle. 
    // All angles in radians
    double MinTurnRads(double init, double final, bool& bOutputReverse);

    // We have to use meters here instead of radians due to the fact that
    // ProfiledPIDController's constraints only take in meters per second and
    // meters per second squared.
    static constexpr radians_per_second_t kModuleMaxAngularVelocity = radians_per_second_t(wpi::math::pi);                                           // radians per second
    static constexpr unit_t<radians_per_second_squared_t> kModuleMaxAngularAcceleration = unit_t<radians_per_second_squared_t>(wpi::math::pi * 2.0); // radians per second squared

    double m_offset;
    std::string m_name;

    CANSparkMax m_driveMotor;
    CANSparkMax m_turningMotor;

    CANPIDController m_drivePIDController = m_driveMotor.GetPIDController();
    CANPIDController m_turnPIDController = m_turningMotor.GetPIDController();

    /// PID param loader for the drive NEO
    PIDLoaderNEO m_drivePIDLoader;
    /// PID param loader for the turn NEO
    PIDLoaderNEO m_turnPIDLoader;

    CANEncoder m_driveEncoder;
    CANEncoder m_turnRelativeEncoder = m_turningMotor.GetEncoder();
    AnalogInput m_turningEncoder;
};