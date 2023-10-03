package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeIOSim implements IntakeIO {
    DCMotor m_motorSim;
    double m_voltage;
    boolean m_brakeEnabled;

    public IntakeIOSim() {
        m_motorSim = DCMotor.getNEO(1);
        m_voltage = 0;
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intakeSpeed = getSpeed();
    }

    @Override
    public void setVoltage(double voltage) {
        m_voltage = voltage;
    }

    @Override
    public double getSpeed() {
        return m_motorSim.getSpeed(40, m_voltage);
    }

    @Override
    public boolean hasGamePiece() {
        // implementation in frc-23 used robotstate and i dont like it
        return false;
    }
}
