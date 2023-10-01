package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleIOSim implements SwerveModuleIO {

    private SwerveModuleState m_curState;
    private SwerveModuleState m_desState;
    private SwerveModulePosition m_curPos;

    public SwerveModuleIOSim() {
        m_curState = new SwerveModuleState();
        m_desState = new SwerveModuleState();
        m_curPos = new SwerveModulePosition();        
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        double oldAngleRads = m_curPos.angle.getRadians();
        double updatedAngle = MathUtil.interpolate(m_curState.angle.getRadians(), m_desState.angle.getRadians(), 1);
        m_curState.angle = Rotation2d.fromRadians(updatedAngle);
        m_curState.speedMetersPerSecond = m_desState.speedMetersPerSecond;

        m_curPos.distanceMeters += (m_curState.speedMetersPerSecond * 0.02);
        m_curPos.angle = m_curState.angle;
        inputs.turnAngle = m_curPos.angle;

        inputs.driveDistanceMeters = m_curPos.distanceMeters;
        inputs.driveVelocityMetersPerSecond = m_curState.speedMetersPerSecond;
        inputs.turnRadsPerSecond = (m_curPos.angle.getRotations() - oldAngleRads) / 0.02;

        inputs.xDriveVelocityMetersPerSecond = Math.cos(inputs.turnAngle.getRadians()) * inputs.driveVelocityMetersPerSecond;
        inputs.yDriveVelocityMetersPerSecond = Math.sin(inputs.turnAngle.getRadians()) * inputs.driveVelocityMetersPerSecond;
    }

    @Override
    public SwerveModulePosition getPosition() {
        return m_curPos;
    }

    @Override
    public void resetDistance() {
        m_curPos.distanceMeters = 0;
    }

    @Override
    public void syncTurningEncoder() {        
    }

    @Override
    public void resetEncoders() {        
    }

    @Override
    public Rotation2d getAngle() {
        return m_curPos.angle;
    }

    @Override
    public void setDesiredState(SwerveModuleState swerveModuleState) {
        m_desState = swerveModuleState;
    }

    @Override
    public SwerveModuleState getState() {
        return m_curState;
    }

    @Override
    public SwerveModuleState getAbsoluteState() {
        return getState();
    }
    
}