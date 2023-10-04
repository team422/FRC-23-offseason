package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeIOCANSparkMax implements IntakeIO {
    private CANSparkMax m_intakeMotor;
    private RelativeEncoder m_encoder;

    public IntakeIOCANSparkMax(int port, double gearRatio) {
        m_intakeMotor = new CANSparkMax(port, CANSparkMax.MotorType.kBrushless);
        m_encoder = m_intakeMotor.getEncoder();
        m_encoder.setPositionConversionFactor(gearRatio);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intakeSpeed = getSpeed();
        inputs.intakeOutputCurrent = m_intakeMotor.getOutputCurrent();
        inputs.intakeOutputVoltage = m_intakeMotor.getAppliedOutput();
    }

    @Override
    public void setVoltage(double voltage) {
        m_intakeMotor.setVoltage(voltage);
    }

    @Override
    public double getSpeed() {
        return m_encoder.getVelocity();
    }

    @Override
    public boolean hasGamePiece() {
        return Math.round(m_intakeMotor.getOutputCurrent()) > 15 && Math.round(getSpeed()) == 0;
    }
}