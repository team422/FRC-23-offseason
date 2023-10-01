package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;

public class Wrist extends SubsystemBase {
    private WristIO m_io;
    private ProfiledPIDController m_controller;
    private Rotation2d m_desiredAngle;
    public WristInputsAutoLogged m_inputs;

    public Wrist(WristIO io, ProfiledPIDController controller) {
        m_io = io;
        m_controller = controller;
        m_inputs = new WristInputsAutoLogged();
        m_desiredAngle = Rotation2d.fromDegrees(0);
    }

    public void setBrakeMode(boolean enabled) {
        m_io.setBrakeMode(enabled);
    } 

    public void setAngle(Rotation2d angle) {
        m_desiredAngle = angle;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        double currAngle = m_inputs.angle.getRadians();
        
        double pidVoltage = m_controller.calculate(currAngle, m_desiredAngle.getRadians());

        // only sim rn so feedforward stuff doesnt exist yet
        m_io.setVoltage(pidVoltage);
    }

    public boolean withinTolerance() {
        return Math.abs(m_desiredAngle.getRadians() - m_inputs.angle.getRadians()) < m_controller.getPositionTolerance();
    }
}
