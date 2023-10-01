package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private IntakeIO m_IO;
    public IntakeInputsAutoLogged m_inputs;
    public double m_desiredVoltage;

    public Intake(IntakeIO io) {
        m_IO = io;
        m_inputs = new IntakeInputsAutoLogged();
        m_desiredVoltage = 0;
    }

    public void periodic(){
        m_IO.updateInputs(m_inputs);
        Logger.getInstance().processInputs("Intake", m_inputs);
        m_IO.setDesiredVoltage(m_desiredVoltage);
    }

    public void setDesiredVoltage(double voltage) {
        m_desiredVoltage = voltage;
    }

    public double getSpeed() {
        return m_IO.getSpeed();
    }

    public Command setIntakeVoltage(double voltage) {
        return runEnd(() -> this.setDesiredVoltage(voltage), 
                      () -> this.setDesiredVoltage(0));
    }

    public static final double kIntakeVoltage = 0.1;

    public Command intakeIn() {
        return setIntakeVoltage(kIntakeVoltage);
    }

    public Command intakeOut() {
        return setIntakeVoltage(-kIntakeVoltage);
    }
}
