package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem {

    private IntakeIO m_IO;
    private IntakeInputsAutoLogged m_inputs;

    public Intake() {
        // m_IO = io;
        // m_inputs = new IntakeInputsAutoLogged();
    }

    public void periodic(){
        m_IO.updateInputs(m_inputs);
    }

    public void setSpeed(double speed){}

    public void brake() {
    }

    public double getSpeed() {
        return 0;
    }
}
