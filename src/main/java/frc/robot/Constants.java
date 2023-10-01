// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean tuningMode = true;

    public static final class DriveConstants {
        public static final class Pigeon2AccelConstants {
            public static final double kMultiplier = 1;
            public static final int kFixedShift = 14;
            public static final int kX = 0;
            public static final int kY = 1;
            public static final int kZ = 2;
        }

        // wheel dims
        public static final double kWheelDiameter = Units.inchesToMeters(3.7);
        public static final double kWheelBase = Units.inchesToMeters(17.25);
        public static final double kTrackWidth = Units.inchesToMeters(17.25);

        // module offsets
        public static Translation2d[] kModuleTranslations = {
                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // front left
                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // front right
                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // rear left
                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0) // rear right
        };

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kModuleTranslations);
        public static final Rotation2d pitchAngle = Rotation2d.fromDegrees(0);
        public static final Pose2d startPose = new Pose2d(3, 5, new Rotation2d());;

        public static final double kDriveDeadband = 0;
        public static final double kMaxAngularSpeedRadiansPerSecond = Units.degreesToRadians(450);
        public static final double kMaxSpeedMetersPerSecond = 3.85;
    }

    public static final class ModuleConstants {
        public static final TunableNumber kDriveP = new TunableNumber("Drive P", 0.1);
        public static final TunableNumber kDriveI = new TunableNumber("Drive I", 0.0);
        public static final TunableNumber kDriveD = new TunableNumber("Drive D", 0.00);
        public static final TunableNumber kDriveFF = new TunableNumber("Drive FF", 2.96);

        public static final TunableNumber kTurningP = new TunableNumber("TurnP", 0.6);
        public static final TunableNumber kTurningI = new TunableNumber("Turn I", 0.00);
        public static final TunableNumber kTurningD = new TunableNumber("Turn D", 0.005);

    }

    public final static class WristConstants {
        // TODO: copied from frc-23, CHANGE BEFORE DEPLOYING TO ROBOT

        public final static double kGearRatio = 34.8444444444;
        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(95);
        public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(-100);
    }

    public static final class Ports {
        public static final int pigeonPort = 422;

        // Left Front Ports
        public static final int leftFrontDrivingMotorPort = 401;
        public static final int leftFrontTurningMotorPort = 69;
        public static final int leftFrontCanCoderPort = 42;

        // Right Front Ports
        public static final int rightFrontDriveMotorPort = 666;
        public static final int rightFrontTurningMotorPort = 444116;
        public static final int rightFrontCanCoderPort = 420;

        // Left Rear Ports
        public static final int leftRearDriveMotorPort = 422;
        public static final int leftRearTurningMotorPort = 422;
        public static final int leftRearCanCoderPort = 422;

        // Right Rear Ports
        public static final int rightRearDriveMotorPort = 422;
        public static final int rightRearTurningMotorPort = 422;
        public static final int rightRearCanCoderPort = 422;

    }

    public static final class OIConstants {
        public static final int kDriverLeftDriveStickPort = 0;
        public static final int kDriverRightDriveStickPort = 1;
    }
}