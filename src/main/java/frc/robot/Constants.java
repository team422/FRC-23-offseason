// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
        public static final double kWheelDiameter = Units.inchesToMeters(4);
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

        // currently limiting speed to 40% for start of testing so we dont break anything if ports are wrong
        public static final double kMaxAngularSpeedMultiplier = 0.40;
        public static final double kMaxAngularSpeedRadiansPerSecond = Units.degreesToRadians(450) * kMaxAngularSpeedMultiplier;
        public static final double kMaxSpeedMultiplier = 0.40;
        public static final double kMaxSpeedMetersPerSecond = 3.85 * kMaxSpeedMultiplier;
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
        // TODO: copied from frc-23, change later
        public static final double kGearRatio = 1/ 0.0397;
        public static final int kWristEncoderCPR = 4096;
        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(95);
        public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(-100);
        public static final double kToleranceRad = Units.degreesToRadians(2);
        public static final double kOffset = 0.0;

        // Wrist PID, currently untuned
        public static final TunableNumber kP = new TunableNumber("Wrist P", 1);
        public static final TunableNumber kI = new TunableNumber("Wrist I", 1);
        public static final TunableNumber kD = new TunableNumber("Wrist D", 1);
        public static final TunableNumber kWristVelo = new TunableNumber("Wrist Velo", 20);
        public static final TunableNumber kWristAccel = new TunableNumber("Wrist Accel", 15.0);
        public static final ProfiledPIDController wristPIDController = new ProfiledPIDController(
            kP.get(), kI.get(), kD.get(),
            new Constraints(kWristVelo.get(), kWristAccel.get()));

        // Wrist Feedforward, currently untuned
        public static final TunableNumber kWristks = new TunableNumber("Wrist ks", 0.05);
        public static final TunableNumber kWristkg = new TunableNumber("Wrist kg", 0.6);
        public static final TunableNumber kWristkv = new TunableNumber("Wrist kv", 0.0);
        public static final ArmFeedforward wristFeedforward = new ArmFeedforward(
            kWristkg.get(), kWristkv.get(), kWristks.get());
    }

    public static final class IntakeConstants {
        // TODO: copied from frc-23, change later
        public static final double kGearRatio = 0.25;

        public static final double kIntakeVoltage = 11;
        public static final double kIntakeHoldVoltage = 3;
    }

    public static final class Ports {
        public static final int pigeonPort = 16;

        // OI Ports
        public static final int kDriverLeftDriveStickPort = 0;
        public static final int kDriverRightDriveStickPort = 1;

        // Left Front Ports
        public static final int leftFrontDrivingMotorPort = 1;
        public static final int leftFrontTurningMotorPort = 2;
        public static final int leftFrontCanCoderPort = 9;

        // Right Front Ports
        public static final int rightFrontDriveMotorPort = 3;
        public static final int rightFrontTurningMotorPort = 4;
        public static final int rightFrontCanCoderPort = 10;

        // Left Rear Ports
        public static final int leftRearDriveMotorPort = 7;
        public static final int leftRearTurningMotorPort = 8;
        public static final int leftRearCanCoderPort = 11;

        // Right Rear Ports
        public static final int rightRearDriveMotorPort = 5;
        public static final int rightRearTurningMotorPort = 6;
        public static final int rightRearCanCoderPort = 12;

        public static final int wristPortDrive = 13;
        public static final int wristPortFollower = 14;

        public static final int intakePort = 15;

    }

    public static final class Setpoints {
        // TODO: i made these up, change later
        public static final Rotation2d kWristGrabCube = Rotation2d.fromDegrees(0);
        public static final Rotation2d kWristShoot = Rotation2d.fromDegrees(30);
        public static final Rotation2d kWristStow = Rotation2d.fromDegrees(90);
    }
}