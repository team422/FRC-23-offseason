package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private SwerveModuleState m_currState;
    private SwerveModuleState m_desiredState;
    private SwerveModulePosition m_currPos;

    public SwerveModuleIOSim() {
        m_currState = new SwerveModuleState();
        m_desiredState = new SwerveModuleState();
        m_currPos = new SwerveModulePosition();
    }

    public SwerveModulePosition getPosition() {
        return m_currPos;
    }

    public void resetDistance() {
        m_currPos.distanceMeters = 0;
    }

    public void syncTurningEncoder() {
    }

    public void resetEncoders() {
    }

    public Rotation2d getAngle() {
        return m_currPos.angle;
    }

    public void setDesiredState(SwerveModuleState swerveModuleState) {
        m_desiredState = swerveModuleState;
    }

    public SwerveModuleState getState() {
        return m_currState;
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        double oldAngleRads = m_currPos.angle.getRadians();
        double updatedAngle = MathUtil.interpolate(m_currState.angle.getRadians(), m_desiredState.angle.getRadians(), 1);
        m_currState.angle = Rotation2d.fromRadians(updatedAngle);
        m_currState.speedMetersPerSecond = m_desiredState.speedMetersPerSecond;

        m_currPos.distanceMeters += m_currState.speedMetersPerSecond * 0.02;
        m_currPos.angle = m_currState.angle;

        inputs.turnAngleRads = m_currPos.angle.getRadians();
        inputs.driveDistanceMeters = m_currPos.distanceMeters;
        inputs.driveVelocityMetersPerSecond = m_currState.speedMetersPerSecond;
        inputs.turnRadsPerSecond = (m_currPos.angle.getRotations() - oldAngleRads) / 0.02;

        inputs.xDriveVelocityMetersPerSecond = Math.cos(inputs.turnAngleRads) * inputs.driveVelocityMetersPerSecond;
        inputs.yDriveVelocityMetersPerSecond = Math.sin(inputs.turnAngleRads) * inputs.driveVelocityMetersPerSecond;
    }

    @Override
    public SwerveModuleState getAbsoluteState() {
        return getState();
    }    
}
