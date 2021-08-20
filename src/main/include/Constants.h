/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <wpi/math>

#include <ctre/phoenix/CANifier.h>

using namespace ctre::phoenix;
using namespace units;

/// Uncomment to set button binds for secondary controller to the primary controller
#define DualJoysticks

namespace DriveConstants
{
    constexpr int kNumSwerveModules = 4;

    /// Distance between centers of left and right wheels on robot
    constexpr meter_t kTrackWidth = 23.5_in;
    /// Distance between centers of front and back wheels on robot
    constexpr meter_t kWheelBase = 23.5_in;

    /// \name Teleop Drive Constraints
    constexpr auto kDriveSpeed = meters_per_second_t(3.5);
    constexpr auto kDriveAngularSpeed = radians_per_second_t(wpi::math::pi * 2.0);

    /// \name CAN bus IDs
    ///@{
    /// CAN IDs for swerve modules
    constexpr int kCanifierID = 0;                       //!< CANifier CAN ID (for absolute encoder PWM inputs)
    
    constexpr int kFrontLeftDriveMotorPort    = 1;       //!< Front Left Drive CAN ID (TalonFX)   
    constexpr int kFrontLeftTurningMotorPort  = 2;       //!< Front Left Turn CAN ID (SparkMAX)   

    constexpr int kFrontRightDriveMotorPort   = 3;       //!< Front Right Drive CAN ID (TalonFX)   
    constexpr int kFrontRightTurningMotorPort = 4;       //!< Front Right Turn CAN ID (SparkMAX)

    constexpr int kRearRightDriveMotorPort    = 5;       //!< Rear Right Drive CAN ID (TalonFX)   
    constexpr int kRearRightTurningMotorPort  = 6;       //!< Rear Right Turn CAN ID (SparkMAX)

    constexpr int kRearLeftDriveMotorPort     = 7;       //!< Rear Left Drive CAN ID (TalonFX)   
    constexpr int kRearLeftTurningMotorPort   = 8;       //!< Rear Left Turn CAN ID (SparkMAX)
    ///@}

    /// \name Canifier PWM channels
    ///@{
    /// PWM channels for the canifier
    constexpr CANifier::PWMChannel kFrontLeftPWM = CANifier::PWMChannel::PWMChannel0;
    constexpr CANifier::PWMChannel kFrontRightPWM = CANifier::PWMChannel::PWMChannel2;
    constexpr CANifier::PWMChannel kRearRightPWM = CANifier::PWMChannel::PWMChannel1;
    constexpr CANifier::PWMChannel kRearLeftPWM = CANifier::PWMChannel::PWMChannel3;
    ///@}

    /// \name Drive wheel reversal (inverting) flags
    ///@{
    /// To keep the swerve module bevel gear facing inwards we need to reverse the right side
    constexpr bool kFrontLeftDriveMotorReversed  = true;
    constexpr bool kRearLeftDriveMotorReversed   = true;
    constexpr bool kFrontRightDriveMotorReversed = false;
    constexpr bool kRearRightDriveMotorReversed  = false;
    ///@}

    constexpr bool kGyroReversed = false;

    // Process for reentering values: 0 all values out, line up with stick, all gears face inwards
    // Line up based on side, left or right
    // Record values, enter below, then redeploy
    // All gears should face outwards

    //============================================LEAVE THESE ZEROES COMMENTED OUT!!!
    // constexpr double kFrontLeftOffset   = 0.0;
    // constexpr double kFrontRightOffset  = 0.0;
    // constexpr double kRearRightOffset   = 0.0;
    // constexpr double kRearLeftOffset    = 0.0;
    //===============================================================================
    constexpr double kFrontLeftOffset   = 2689.0;
    constexpr double kFrontRightOffset  = 205.0;
    constexpr double kRearRightOffset   = 1858.0;
    constexpr double kRearLeftOffset    = 983.0;

    // Pulse Width per rotation is not equal for all encoders. Some are 0 - 3865, some are 0 - 4096
    // FL: 4096
    // FR: 3970
    // RL: 4096
    // RR: 3865
    constexpr double kPulseWidthToZeroOne = 4096.0;    // 4096 micro second pulse width is full circle
    constexpr double kPulseWidthToRadians =  2.0 * wpi::math::pi / kPulseWidthToZeroOne;

    /// \name Robot RotationDrive PID Controller
    ///@{
    /// Rotation PID Controller for Rotation Drive, converts between radians angle error to radians per second turn
    constexpr double kRotationDriveP = 1;
    constexpr double kRotationDriveI = 0;
    constexpr double kRotationDriveIMaxRange = 0;
    constexpr double kRotationDriveD = 0.025;
    /// Max speed for control
    constexpr double kRotationDriveMaxSpeed = 3.5;
    /// Speeds higher than value will prevent robot from changing directions for a turn
    constexpr double kRotationDriveDirectionLimit = 3;
    /// Tolerance for turning
    constexpr double kRotationDriveTolerance = 0.07;
    ///@}
}

namespace ModuleConstants
{
    constexpr int kEncoderCPR = 2048;

    constexpr int kEncoderTicksPerSec = 10;                 //!< TalonFX::GetSelectedSensorVelocity() returns ticks/100ms = 10 ticks/sec
    constexpr double kWheelDiameterMeters = .1016;          //!< 4"

    constexpr double kDriveGearRatio = 8.16;                //!< MK3 swerve modules w/NEOs 12.1 ft/sec w/Falcon 13.6 ft/sec
    constexpr double kTurnMotorRevsPerWheelRev = 12.8;
    /// Assumes the encoders are directly mounted on the wheel shafts
    /// ticks / 100 ms -> ticks / s -> motor rev / s -> wheel rev / s -> m / s
    constexpr double kDriveEncoderMetersPerSec = kEncoderTicksPerSec / static_cast<double>(kEncoderCPR) / kDriveGearRatio * (kWheelDiameterMeters * wpi::math::pi);

    constexpr double kTurnEncoderCPR = 4096.0 / kTurnMotorRevsPerWheelRev;    // Mag encoder relative output to SparkMax

    constexpr double kP_ModuleTurningController = 1.1;

    constexpr double kD_ModuleTurningController = 0.03;
    constexpr double kPModuleDriveController = 0.001;

    constexpr uint kMotorCurrentLimit = 30;

    /// \name Turn PID Controller for Swerve Modules
    ///@{
    constexpr bool kTurnAdjust = false;
    constexpr double kTurnP = 0.75;
    constexpr double kTurnI = 0.0;
    constexpr double kTurnD = 0.0;
    constexpr double kTurnIA = 0.0;
    constexpr double kTurnIZ = 0.0;
    ///@}

    /// \name Drive PID Controller for Swerve Modules
    ///@{
    constexpr bool kDriveAdjust = false;
    constexpr double kDriveP = 0.1;
    constexpr double kDriveI = 0;
    constexpr double kDriveD = 0;
    constexpr double kDriveFF = 0.047619;
    ///@}
}

namespace AutoConstants
{
    using radians_per_second_squared_t = compound_unit<radians, inverse<squared<second>>>;

    constexpr auto kMaxSpeed = meters_per_second_t(3.75);
    constexpr auto kMaxAcceleration = meters_per_second_squared_t(4.5);
    constexpr auto kMaxAngularSpeed = radians_per_second_t(wpi::math::pi * 6.0);
    constexpr auto kMaxAngularAcceleration = unit_t<radians_per_second_squared_t>(wpi::math::pi * 6.0);

    constexpr double kPXController = 7.0;
    constexpr double kDXController = 0.7;
    constexpr double kPYController = 7.0;
    constexpr double kDYController = 0.7;
    constexpr double kPThetaController = 10.0;
    constexpr double kDThetaController = 0.9;

    extern const frc::TrapezoidProfile<radians>::Constraints kThetaControllerConstraints;
}  // namespace AutoConstants

namespace OIConstants
{
    constexpr double kDeadzoneX = 0.015;
    constexpr double kDeadzoneY = 0.015;
    constexpr double kDeadzoneXY = 0.08;
    constexpr double kDeadzoneRot = 0.10;
    constexpr double kDeadzoneAbsRot = 0.50;

    constexpr int kPrimaryControllerPort = 0;
#ifdef DualJoysticks
    constexpr int kSecondaryControllerPort = 1;
#else
    constexpr int kSecondaryControllerPort = 0;
#endif
}

// Vision Subsystem Constants
namespace VisionConstants
{
    // 6/30/21
    // Limelight X Offset: -0.04
    // Mounting angle of the limelight, in degrees
    constexpr double kMountingAngle = 25.0;
    // Permanent X adjustment -0.05
    // Mounting height of the limelight from the ground, in inches
    constexpr double kMountingHeight = 22;
    // Target center height, in inches
    // 6/30/21 Changed: Target bottom now instead for consistent tracking in worse conditions
    constexpr double kTargetHeight = 81.25;  //98.25;

    constexpr double kMinTargetDistance = 70;
    constexpr double kMaxTargetDistance = 380;

    constexpr double kMinHoneDistance = 130;
    constexpr double kMaxHoneDistance = 260;
}