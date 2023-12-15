// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.lib.pathplanner.PathPlannerUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.autonomous.AutoFactory;
import frc.robot.commands.autonomous.ChargeStationBalance;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsController;
import frc.robot.oi.DriverControlsDualFlightStick;
import frc.robot.oi.OperatorControls;
import frc.robot.oi.OperatorControlsXbox;
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

    private LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    private AutoFactory m_autoFactory;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureSubsystems();
        configureButtonBindings();
        configureAprilTags();
        configureAuto();
    }

    public void configureAuto(){
        m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
        m_autoFactory = new AutoFactory(m_drive, m_wrist, m_intake);

        // Add basic autonomous commands
        m_autoChooser.addDefaultOption("Do Nothing", Commands.none());
        // m_autoChooser.addDefaultOption("three_wall", Commands.none());

        // Add PathPlanner Auto Commands
        PathPlannerUtil.getExistingPaths().forEach(path -> {
            m_autoChooser.addOption(path, m_autoFactory.getAutoCommand(path));
        });
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
            GyroIOPigeon pigeon = new GyroIOPigeon(Ports.pigeonPort, DriveConstants.pitchAngle);
            m_drive = new Drive(pigeon,
                    new AccelerometerIOWPI(new Pigeon2Accel(Ports.pigeonPort)),
                    DriveConstants.startPose,
                    m_swerveModuleIOs);

            m_intake = new Intake(new IntakeIOCANSparkMax(Ports.intakePort, IntakeConstants.kGearRatio),
                    IntakeConstants.kIntakeVoltage, IntakeConstants.kIntakeHoldVoltage);

            m_wrist = new Wrist(new WristIOCANSparkMax(Ports.wristPortDrive, Ports.wristPortFollower, WristConstants.kOffset),
                    WristConstants.wristPIDController, WristConstants.wristGamepiecePIDController, WristConstants.wristFeedforward,
                    WristConstants.kMinAngle, WristConstants.kMaxAngle, WristConstants.kToleranceRad, WristConstants.kManualMoveVolts);
        } else {
            m_drive = new Drive(new GyroIOPigeon(22, new Rotation2d()), new AccelerometerIOSim(), new Pose2d(),
                    new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim());

            m_intake = new Intake(new IntakeIOSim(),
                    IntakeConstants.kIntakeVoltage, IntakeConstants.kIntakeHoldVoltage);

            m_wrist = new Wrist(new WristIOSim(), WristConstants.wristPIDController, WristConstants.wristGamepiecePIDController, WristConstants.wristFeedforward,
                    WristConstants.kMinAngle, WristConstants.kMaxAngle, WristConstants.kToleranceRad, WristConstants.kManualMoveVolts);
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
        
        DriverControls driverControls = new DriverControlsController(4);
        OperatorControls operatorControls = new OperatorControlsXbox(Ports.kOperatorControllerPort);
        TeleopDrive teleopDrive = new TeleopDrive(m_drive, driverControls::getDriveX,
                driverControls::getDriveY, driverControls::getDriveRotation,
                DriveConstants.kDriveDeadband);
        m_drive.setDefaultCommand(teleopDrive);
        // if driverControls.intakeButton is greater than 0.1 then run m_intake.intakeCommand()
        driverControls.intakeButton().whileTrue(m_intake.intakeCommand());
        driverControls.outtakeButton().whileTrue(m_intake.outtakeFastCommand());
        ChargeStationBalance m_charge = new ChargeStationBalance(m_drive);
        driverControls.balance().whileTrue(m_charge);
        driverControls.manualFieldReset().onTrue(m_drive.manualFieldCentricCommand());
        // if (Robot.isSimulation()) {
            // all of these are unbound in real
            // DO NOT REBIND THESE UNTIL WE HAVE A MIN ANGLE AND MAX ANGLE AND OFFSET IS SET
            driverControls.wristButtonIntake().onTrue(m_wrist.setAngleCommand(Setpoints.kWristGrabCube));
            driverControls.wristButtonStow().onTrue(m_wrist.setAngleCommand(Setpoints.kWristStow));
            operatorControls.wristButtonShootLow().onTrue(m_wrist.setAngleCommand(Setpoints.kWristShootLow));
        // }

        driverControls.wristManualUp().whileTrue(m_wrist.manualUpCommand());
        driverControls.wristManualDown().whileTrue(m_wrist.manualDownCommand());

        operatorControls.wristManualUp().whileTrue(m_wrist.manualUpCommand());
        operatorControls.wristManualDown().whileTrue(m_wrist.manualDownCommand());

        driverControls.resetWristEncoder().onTrue(m_wrist.resetEncoderCommand());

        operatorControls.outtakeFastButton().whileTrue(m_intake.outtakeFastCommand());
        operatorControls.outtakeSlowButton().whileTrue(m_intake.outtakeSlowCommand());

        driverControls.toggleHasGamepiece().onTrue(m_wrist.toggleGamepieceCommand());
        driverControls.wristButtonIntake().whileTrue(Commands.runOnce(() -> System.out.println("he")));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return m_autoChooser.get();
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