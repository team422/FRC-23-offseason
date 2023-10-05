// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.accelerometer.AccelerometerIOSim;
import frc.robot.subsystems.drive.accelerometer.AccelerometerIOWPI;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOCANSparkMax;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOCANSparkMax;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.util.Pigeon2Accel;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Setpoints;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsDualFlightStick;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIOMK4iSparkMax;
import frc.robot.subsystems.drive.SwerveModuleIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private Drive m_drive;
    private Intake m_intake;
    private Wrist m_wrist;
    private AprilTagFieldLayout m_layout;
    // private RobotState m_robotState;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureSubsystems();
        configureButtonBindings();
        configureAprilTags();
    }

    public void configureAprilTags() {
        try {
            m_layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException e) {
            System.out.println("AprilTag field layout not found:" + e);
        }
    }

    private void configureAllianceSettings() {
        var origin = DriverStation.getAlliance() == Alliance.Blue
                ? OriginPosition.kBlueAllianceWallRightSide
                : OriginPosition.kRedAllianceWallRightSide;
        m_layout.setOrigin(origin);
    }

    private void configureSubsystems() {
        if (Robot.isReal()) {
            SwerveModuleIO[] m_swerveModuleIOs = {
                    new SwerveModuleIOMK4iSparkMax(Ports.leftFrontDrivingMotorPort,
                            Ports.leftFrontTurningMotorPort,
                            Ports.leftFrontCanCoderPort),
                    new SwerveModuleIOMK4iSparkMax(Ports.rightFrontDriveMotorPort,
                            Ports.rightFrontTurningMotorPort,
                            Ports.rightFrontCanCoderPort),
                    new SwerveModuleIOMK4iSparkMax(Ports.leftRearDriveMotorPort,
                            Ports.leftRearTurningMotorPort,
                            Ports.leftRearCanCoderPort),
                    new SwerveModuleIOMK4iSparkMax(Ports.rightRearDriveMotorPort,
                            Ports.rightRearTurningMotorPort,
                            Ports.rightRearCanCoderPort) };
            GyroIOPigeon pigeon = new GyroIOPigeon(Ports.pigeonPort, Constants.DriveConstants.pitchAngle);
            m_drive = new Drive(pigeon,
                    new AccelerometerIOWPI(new Pigeon2Accel(Ports.pigeonPort)),
                    Constants.DriveConstants.startPose,
                    m_swerveModuleIOs);

            m_intake = new Intake(new IntakeIOCANSparkMax(Ports.intakePort, IntakeConstants.kGearRatio),
                    IntakeConstants.kIntakeVoltage, IntakeConstants.kIntakeHoldVoltage);

            m_wrist = new Wrist(new WristIOCANSparkMax(Ports.wristPort, WristConstants.kOffset),
                    WristConstants.wristPIDController, WristConstants.wristFeedforward, WristConstants.kMinAngle,
                    WristConstants.kMaxAngle, WristConstants.kToleranceRad);
        } else {
            m_drive = new Drive(new GyroIOPigeon(22, new Rotation2d()), new AccelerometerIOSim(), new Pose2d(),
                    new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim());

            m_intake = new Intake(new IntakeIOSim(),
                    IntakeConstants.kIntakeVoltage, IntakeConstants.kIntakeHoldVoltage);

            m_wrist = new Wrist(new WristIOSim(), WristConstants.wristPIDController, WristConstants.wristFeedforward,
                    WristConstants.kMinAngle, WristConstants.kMaxAngle, WristConstants.kToleranceRad);
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        
        DriverControls driverControls = new DriverControlsDualFlightStick(
            Constants.OIConstants.kDriverLeftDriveStickPort,
            Constants.OIConstants.kDriverRightDriveStickPort);
        TeleopDrive teleopDrive = new TeleopDrive(m_drive, driverControls::getDriveX,
                driverControls::getDriveY, driverControls::getDriveRotation,
                Constants.DriveConstants.kDriveDeadband);
        m_drive.setDefaultCommand(teleopDrive);

        driverControls.intakeButton().whileTrue(m_intake.intakeCommand());
        driverControls.outtakeButton().whileTrue(m_intake.outtakeCommand());

        if (Robot.isSimulation()) {
            // all of these are unbound in real
            // DO NOT REBIND THESE UNTIL WE HAVE A MIN ANGLE AND MAX ANGLE AND OFFSET IS SET
            driverControls.wristButtonCube().onTrue(m_wrist.setAngleCommand(Setpoints.kWristGrabCube));
            driverControls.wristButtonShoot().onTrue(m_wrist.setAngleCommand(Setpoints.kWristShoot));
            driverControls.wristButtonStow().onTrue(m_wrist.setAngleCommand(Setpoints.kWristStow));
        }

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }

    public void onEnabled() {
        configureAllianceSettings();
    }

    public void disabledPeriodic() {
    }

    public void onDisabled() {
    }

    public void updateRobotState() {
    }

}