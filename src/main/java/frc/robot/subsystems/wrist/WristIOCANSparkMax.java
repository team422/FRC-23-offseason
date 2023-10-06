package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;

public class WristIOCANSparkMax implements WristIO {
    private CANSparkMax m_wristLeader;
    private CANSparkMax m_wristFollower;
    private SparkMaxAbsoluteEncoder m_encoder;
    private boolean m_brakeModeEnabled;

    public WristIOCANSparkMax(int leaderPort, int followerPort, double encoderOffset) {
        m_wristLeader = new CANSparkMax(leaderPort, CANSparkMax.MotorType.kBrushless);
        m_wristFollower = new CANSparkMax(followerPort, CANSparkMax.MotorType.kBrushless);
        m_wristFollower.follow(m_wristLeader);
        m_encoder = m_wristLeader.getAbsoluteEncoder(Type.kDutyCycle);
        m_wristLeader.setInverted(false);
        m_wristLeader.setIdleMode(IdleMode.kBrake);
        m_brakeModeEnabled = true;
        m_encoder.setPositionConversionFactor(2 * Math.PI);
        m_encoder.setInverted(true);
        m_encoder.setZeroOffset(encoderOffset);
    }

    @Override
    public void updateInputs(WristInputs inputs) {
        inputs.angleRad = getAngle().getRadians();
        inputs.outputVoltage = getOutputVoltage();
        inputs.currentAmps = m_wristLeader.getOutputCurrent();
        inputs.wristSpeed = getSpeed();
    }

    @Override
    public void setVoltage(double voltage) {
        m_wristLeader.setVoltage(voltage);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        if (enabled){
            m_wristLeader.setIdleMode(IdleMode.kBrake);
            m_brakeModeEnabled = true;
        }
        else {
            m_wristLeader.setIdleMode(IdleMode.kCoast);
            m_brakeModeEnabled = false;

        }
    }
    
    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_encoder.getPosition());
    }

    @Override
    public boolean getBrakeMode() {
        return m_brakeModeEnabled;
    }

    @Override
    public double getSpeed() {
        return m_encoder.getVelocity();
    }

    @Override
    public double getOutputVoltage() {
        return m_wristLeader.getAppliedOutput() * m_wristLeader.getBusVoltage();
    }
        
}