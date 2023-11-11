package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ModuleConstants;

public class SwerveModuleIOSim implements SwerveModuleIO {

    private DCMotorSim m_driveMotor;
    private DCMotorSim m_turnMotor;

    private double m_voltageDrive;
    private double m_voltageTurn;

    private SwerveModuleState m_curState;
    private SwerveModuleState m_desState;
    private SwerveModulePosition m_curPos;

    public SwerveModuleIOSim() {
        m_driveMotor = new DCMotorSim(DCMotor.getNEO(1), 1/20.462, 0.01);
        m_turnMotor = new DCMotorSim(DCMotor.getNEO(1), 21.428, 0.01);        
        m_curState = new SwerveModuleState();
        m_desState = new SwerveModuleState();
        m_curPos = new SwerveModulePosition();
        m_voltageDrive = 0;
        m_voltageTurn = 0;        
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        // OLD SIM CODE
        // double oldAngleRads = m_curPos.angle.getRadians();
        // double updatedAngle = MathUtil.interpolate(m_curState.angle.getRadians(), m_desState.angle.getRadians(), 1);
        // m_curState.angle = Rotation2d.fromRadians(updatedAngle);
        // m_curState.speedMetersPerSecond = m_desState.speedMetersPerSecond;

        // m_curPos.distanceMeters += (m_curState.speedMetersPerSecond * 0.02);
        // m_curPos.angle = m_curState.angle;
        // inputs.turnAngleRad = m_curPos.angle.getRadians();

        // inputs.driveDistanceMeters = m_curPos.distanceMeters;
        // inputs.driveVelocityMetersPerSecond = m_curState.speedMetersPerSecond;
        // inputs.turnRadsPerSecond = (m_curPos.angle.getRotations() - oldAngleRads) / 0.02;

        inputs.turnAngleRad = getAngle().getRadians();
        inputs.turnRadsPerSecond = m_turnMotor.getAngularVelocityRadPerSec();
        
        inputs.driveDistanceMeters = getPosition().distanceMeters;
        inputs.driveVelocityMetersPerSecond = getSpeed();
        inputs.driveAmps = m_driveMotor.getCurrentDrawAmps();

        inputs.xDriveVelocityMetersPerSecond = Math.cos(inputs.turnAngleRad) * inputs.driveVelocityMetersPerSecond;
        inputs.yDriveVelocityMetersPerSecond = Math.sin(inputs.turnAngleRad) * inputs.driveVelocityMetersPerSecond;

        inputs.driveVoltage = m_voltageDrive;
        inputs.turnVoltage = m_voltageTurn;
        
        m_driveMotor.update(0.02);
        m_turnMotor.update(0.02);
    }

    public double getSpeed(){
        return m_driveMotor.getAngularVelocityRadPerSec() * ModuleConstants.kDriveConversionFactor;
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveMotor.getAngularPositionRad() * ModuleConstants.kDriveConversionFactor, 
            getAngle());
    }

    @Override
    public void resetDistance() {
        // m_curPos.distanceMeters = 0;

        m_driveMotor = new DCMotorSim(DCMotor.getNEO(1), ModuleConstants.kDriveConversionFactor, 0.01);
    }

    @Override
    public void syncTurningEncoder() {        
    }

    @Override
    public void resetEncoders() {        
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_turnMotor.getAngularPositionRad() % (2 * Math.PI));
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        double driveOutput = state.speedMetersPerSecond;
        setVoltage(driveOutput, 0.0);

    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), getAngle());
    }

    @Override
    public SwerveModuleState getAbsoluteState() {
        return getState();
    }

    @Override
    public void setVoltage(double driveVoltage, double turnVoltage) {
        m_voltageDrive = driveVoltage;
        m_voltageTurn = turnVoltage;
        m_driveMotor.setInputVoltage(driveVoltage);
        m_turnMotor.setInputVoltage(turnVoltage);
    }

    @Override
    public double getDriveVoltage() {
        // TODO: change
        return 0.0;
    }

    @Override
    public double getTurnVoltage() {
        // TODO: change
        return 0.0;
    }
    
}