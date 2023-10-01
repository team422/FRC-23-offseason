// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static class DriveConstants {
        // TODO: copied from frc-23, CHANGE BEFORE DEPLOYING TO ROBOT
        public static final double kDriveDeadband = 0.1;

        public static final double kWheelDiameter = Units.inchesToMeters(3.7);

        public static final double kWheelBase = Units.inchesToMeters(23);
        public static final double kTrackWidth = Units.inchesToMeters(23);
        public static Translation2d[] kModuleTranslations = {
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // front left
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // front right
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // rear left
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0) // rear right
        };
        public static final PIDController turnHeadingPID = new PIDController(0.1, 0, 0);

        public static final Pose2d startPose = new Pose2d(3, 5, new Rotation2d());
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kModuleTranslations);
        public static final double kMaxModuleSpeedMetersPerSecond = 6;
        public static final double kMaxSpeedMetersPerSecond = 8.5; // 8.5
        public static final double kMaxHighElevatorSpeedMetersPerSecond = 1.8; // 8.5
        public static final double kMaxAccelMetersPerSecondSq = 4;

        public static final double kMaxAcceptedErrorMeters = 0.5;
        public static final Rotation2d kMaxAcceptedAngleError = Rotation2d.fromDegrees(10);

        public static final double kMaxSpeedMetersPerSecondAuto = 4; // 3.85 is correct 2023-04-01
        public static final double kMaxAccelMetersPerSecondSqAuto = 3.3;

        public static final double kMaxAngularSpeedRadiansPerSecond = Units.degreesToRadians(360);
        public static final double kMaxHighElevatorAngularSpeedRadiansPerSecond = Units.degreesToRadians(240);

        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Units.degreesToRadians(180);
        public static final Rotation2d pitchAngle = Rotation2d.fromDegrees(-1.17);
    }

    public final static class WristConstants {
        // TODO: copied from frc-23, CHANGE BEFORE DEPLOYING TO ROBOT
        public final static int kNumSimMotors = 4;
        public final static double kGearRatio = 34.8444444444;
        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(95);
        public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(-100);
    }
}
