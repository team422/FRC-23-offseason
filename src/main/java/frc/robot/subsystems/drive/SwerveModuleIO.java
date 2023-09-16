package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO extends LoggedIO<SwerveModuleIO.SwerveModuleInputs> {
    @AutoLog
    public static class SwerveModuleInputs {
        public double turnAngleRads;
        public double turnRadsPerSecond;
        public double driveDistanceMeters;
        public double driveVelocityMetersPerSecond;
    }
    
    public SwerveModulePosition getPosition();

    public default void setUpModuleFirmware() {
    };

    public void resetDistance();

    public void syncTurningEncoder();

    public void resetEncoders();

    public Rotation2d getAngle();

    public void setDesiredState(SwerveModuleState swerveModuleState);

    public SwerveModuleState getState();

    public SwerveModuleState getAbsoluteState();
}
