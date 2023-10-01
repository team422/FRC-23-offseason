package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private IntakeIO m_io;
    public IntakeInputsAutoLogged m_inputs;

    public Intake(IntakeIO io) {
        m_io = io;
        m_inputs = new IntakeInputsAutoLogged();
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
    }

    public void setVoltage(double voltage) {
        m_io.setVoltage(voltage);
    }

    public double getSpeed() {
        return m_inputs.intakeSpeed;
    }
}
