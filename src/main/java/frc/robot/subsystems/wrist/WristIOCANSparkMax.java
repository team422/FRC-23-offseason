package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;

public class WristIOCANSparkMax implements WristIO {
    private CANSparkMax m_wristMotor;
    private SparkMaxAbsoluteEncoder m_encoder;
    private boolean m_brakeModeEnabled;

    public WristIOCANSparkMax(int port, double encoderOffset) {
        m_wristMotor = new CANSparkMax(port, CANSparkMax.MotorType.kBrushless);
        m_encoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_wristMotor.setInverted(false);
        m_wristMotor.setIdleMode(IdleMode.kBrake);
        m_brakeModeEnabled = true;
        m_encoder.setPositionConversionFactor(2 * Math.PI);
        m_encoder.setInverted(true);
        m_encoder.setZeroOffset(encoderOffset);
    }

    @Override
    public void updateInputs(WristInputs inputs) {
        inputs.angleRad = getAngle().getRadians();
        inputs.outputVoltage = getOutputVoltage();
        inputs.currentAmps = m_wristMotor.getOutputCurrent();
        inputs.wristSpeed = getSpeed();
    }

    @Override
    public void setVoltage(double voltage) {
        m_wristMotor.setVoltage(voltage);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        if (enabled){
            m_wristMotor.setIdleMode(IdleMode.kBrake);
            m_brakeModeEnabled = true;
        }
        else {
            m_wristMotor.setIdleMode(IdleMode.kCoast);
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
        return m_wristMotor.getAppliedOutput() * m_wristMotor.getBusVoltage();
    }
        
}