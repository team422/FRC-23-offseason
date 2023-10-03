package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.wrist.WristIO.WristInputs;

public class Wrist extends SubsystemBase {
    private final WristIO m_io;
    public final WristInputs m_inputs;

    private final ProfiledPIDController m_controller;
    private final Rotation2d kMinAngle;
    private final Rotation2d kMaxAngle;
    private Rotation2d m_desiredAngle;

    public Wrist(WristIO io, ProfiledPIDController controller, Rotation2d minAngle, Rotation2d maxAngle, double toleranceRad) {
        m_controller = controller;
        m_controller.setTolerance(toleranceRad);
        m_io = io;
        kMaxAngle = maxAngle;
        kMinAngle = minAngle;
        m_desiredAngle = Rotation2d.fromDegrees(0);
        m_inputs = new WristInputsAutoLogged();
    }
    
    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        double currAngle = m_inputs.angle.getRadians();
        double pidVoltage = m_controller.calculate(currAngle, m_desiredAngle.getRadians());
        m_io.setVoltage(pidVoltage);
    }

    public void setAngle(Rotation2d angle) {
        m_desiredAngle = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), kMinAngle.getRadians(), kMaxAngle.getRadians()));
    }

    public void setBrakeMode(boolean enabled) {
        m_io.setBrakeMode(enabled);
    }
    
    public Rotation2d getAngle() {
        return m_inputs.angle;
    }

    public boolean withinTolerance() {
        return Math.abs(m_desiredAngle.getRadians() - m_inputs.angle.getRadians()) < m_controller.getPositionTolerance();
    }

    public boolean withinTolerance(double toleranceRad) {
        return Math.abs(m_desiredAngle.getRadians() - m_inputs.angle.getRadians()) < toleranceRad;
    }

    public Command setAngleCommand(Rotation2d angle) {
        return run(() -> setAngle(angle)).until(this::withinTolerance);
    }

}
