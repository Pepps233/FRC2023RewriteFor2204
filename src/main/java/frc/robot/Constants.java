// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static final double wheelCircumference = 0.5;
    public static final double gearRatio = 10;
    public static final class ModuleConstants {
        // I don't know the values
        public static final double kP = 0.5;
        public static final int timeOutMs = 50;
        public static final double PositionsToRadiansConversion = ((2 * Math.PI) / 4096);
        // assuming wheel circumference is 0.5 meters and gear ratio is 10:1
        public static final double FalconToMetersPerSecondConversion = (wheelCircumference * 2 * Math.PI) / (gearRatio * 60);
    }

    public static final class DriveConstants {
        // distance between right and left wheels (couldn't find value)
        public static final double kTrackWidth = Units.inchesToMeters(21);
        // distance between front and back wheels (couldn't find value)
        public static final double kWheelBase = Units.inchesToMeters(25.5);

        /*
            creating new SwerveDriveKinematics object with locations of all four swerve modules for wpi library to
            set up the geometry and do the calculations
        */
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
        public static final double MaxSpeedMetersPerSecond = 5;
        public static final double MaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kBackLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackRightDriveMotorPort = 4;
        public static final int kFrontLeftTurningMotorPort = 5;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kFrontRightTurningMotorPort = 7;
        public static final int kBackRightTurningMotorPort = 8;
        // all turning encoder reversed are set to true
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRighturningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;
        // only left encoder reversed are set to true
        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;
        // absolute encoder ports
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 1;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;
        // all absolute encoder reversed are set to false
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
        // absolute encoder offsets in radians (used values from 2204 in frc 2023)
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 306.29;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 22.76;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 358.59;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 30.32;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxSpeedMetersPerSecond = MaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = MaxAngularSpeedRadiansPerSecond / 4;
    }

    public static final class OIConstants {
        // mappings: ex. kDriverYAxis = 1 corresponds to the right gamepad axis
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
        public static final int kDriverControllerPort = 0;
        public static final double kDeadband = 0.05;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.MaxSpeedMetersPerSecond / 4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kPXController = 1.2;
        public static final double kPYController = 1.2;
        public static final double kPThetaController = 2.7;
        public static final double kMaxAngularSpeedRadiansPerSecond =
                DriveConstants.MaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final TrapezoidProfile.Constraints kThetaControllerConstrains =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);

    }
}
