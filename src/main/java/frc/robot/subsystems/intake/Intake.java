package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeInputsAutoLogged;

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

    public void setSpeed(double speed) {
        m_io.setVoltage(speed);
    }

    public void brake(boolean enabled) {
        m_io.brake(enabled);
    }
}
