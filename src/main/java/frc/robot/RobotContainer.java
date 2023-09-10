// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.oi.DriverControls;
import frc.robot.oi.DriverControlsDualFlightStick;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.accelerometer.AccelerometerIOWPI;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;
import frc.robot.util.Pigeon2Accel;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIOMK4iSparkMax;

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
    // private RobotState m_robotState;

    /**
            * The container for the robot. Contains subsystems, OI devices, and commands.
            */
    public RobotContainer() {
            // Configure the button bindings
            configureButtonBindings();
            configureSubsystems();
    }

    private void configureSubsystems() {
        if (Robot.isReal()) {
            SwerveModuleIO[] m_swerveModuleIOs = {
                new SwerveModuleIOMK4iSparkMax(Constants.Ports.leftFrontDrivingMotorPort,
                    Ports.leftFrontTurningMotorPort,
                    Ports.leftFrontCanCoderPort),
                new SwerveModuleIOMK4iSparkMax(Constants.Ports.rightFrontDriveMotorPort,
                    Ports.rightFrontTurningMotorPort,
                    Ports.rightFrontCanCoderPort),
                new SwerveModuleIOMK4iSparkMax(Constants.Ports.leftRearDriveMotorPort,
                    Ports.leftRearTurningMotorPort,
                    Ports.leftRearCanCoderPort),
                new SwerveModuleIOMK4iSparkMax(Constants.Ports.rightRearDriveMotorPort,
                    Ports.rightRearTurningMotorPort,
                    Ports.rightRearCanCoderPort) };
            GyroIOPigeon pigeon = new GyroIOPigeon(Constants.Ports.pigeonPort, Constants.DriveConstants.pitchAngle);
            m_drive = new Drive(pigeon,
                new AccelerometerIOWPI(new Pigeon2Accel(Constants.Ports.pigeonPort)),
                Constants.DriveConstants.startPose,
                m_swerveModuleIOs);
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
        TeleopDrive teleopDrive = new TeleopDrive(m_drive, driverControls::getDriveForward,
            driverControls::getDriveLeft, driverControls::getDriveRotation,
            Constants.DriveConstants.kDriveDeadband);
        m_drive.setDefaultCommand(teleopDrive);
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
    }

    public void disabledPeriodic() {
    }

    public void onDisabled() {
    }

    public void updateRobotState() {
    }

}