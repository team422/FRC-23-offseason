package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class IntakeIOCANSparkMax implements IntakeIO {
    private CANSparkMax m_intakeMotor;
    // private RelativeEncoder m_encoder;
    private SparkMaxAbsoluteEncoder m_encoder;
    private double m_voltage;
    public IntakeIOCANSparkMax(int port, double gearRatio) {
        m_intakeMotor = new CANSparkMax(port, CANSparkMax.MotorType.kBrushless);
        // m_encoder = m_intakeMotor.getEncoder();
        m_encoder = m_intakeMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_encoder.setPositionConversionFactor(gearRatio);
        m_voltage = 0 ;
        
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intakeSpeed = getSpeed();
        inputs.intakeOutputCurrent = m_intakeMotor.getOutputCurrent();
        inputs.intakeOutputVoltage = m_voltage;
        Logger.getInstance().recordOutput("Intake/FrameCurrentResistance", m_intakeMotor.getOutputCurrent() /Math.max(1,m_voltage));
    }

    @Override
    public void setVoltage(double voltage) {
        m_voltage = voltage;
        m_intakeMotor.setVoltage(voltage);
    }

    @Override
    public double getSpeed() {
        return m_encoder.getPosition();
        // return m_encoder.getVelocity();
    }

    @Override
    public Boolean hasGamePiece() {
        if(m_intakeMotor.getOutputCurrent() /Math.max(1,m_voltage) == 0){
            return null;
        }
        return (m_intakeMotor.getOutputCurrent() /Math.max(1,m_voltage)) > 15 && Math.round(getSpeed()) == 0;
    }
}