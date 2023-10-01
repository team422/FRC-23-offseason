package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeIOCANSparkMax implements IntakeIO {
    private CANSparkMax m_intakeMotor;
    private RelativeEncoder m_encoder;

    public IntakeIOCANSparkMax(int port, int intakeEncoderCPR) {
        m_intakeMotor = new CANSparkMax(port, CANSparkMax.MotorType.kBrushless);
        m_encoder = m_intakeMotor.getEncoder();
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intakeSpeed = getSpeed();
        inputs.intakeOutputCurrent = m_intakeMotor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double voltage) {
        m_intakeMotor.setVoltage(voltage);
    }

    @Override
    public double getSpeed() {
        return m_encoder.getVelocity();
    }
}