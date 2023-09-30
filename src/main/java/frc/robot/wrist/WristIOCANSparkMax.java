package frc.robot.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;

public class WristIOCANSparkMax implements WristIO {
    private CANSparkMax m_wristMotor;
    private SparkMaxAbsoluteEncoder m_encoder;
    private boolean m_brakeModeEnabled;

    public WristIOCANSparkMax(int port, int wristEncoderCPR) {
        m_wristMotor = new CANSparkMax(port, CANSparkMax.MotorType.kBrushless);
        m_encoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_wristMotor.setInverted(false);
        m_wristMotor.setIdleMode(IdleMode.kBrake);
        m_brakeModeEnabled = true;
        m_encoder.setPositionConversionFactor(2 * Math.PI);
        m_encoder.setInverted(true);
    }

    @Override
    public void updateInputs(WristInputs inputs) {
        inputs.angle = getAngle();
        inputs.outputVoltage = m_wristMotor.getAppliedOutput() * m_wristMotor.getBusVoltage();
        inputs.currentAmps = m_wristMotor.getOutputCurrent();
        inputs.velocity = m_encoder.getVelocity();
    }

    @Override
    public void setVoltage(double voltage) {
        m_wristMotor.setVoltage(voltage);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        m_wristMotor.setIdleMode(null);
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
        
}



