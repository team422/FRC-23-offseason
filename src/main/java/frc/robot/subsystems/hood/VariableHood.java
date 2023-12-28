package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VariableHood extends SubsystemBase {
    private HoodInputsAutoLogged m_inputs;
    private HoodIO m_io;
    private ProfiledPIDController m_controller;
    private final Rotation2d kMinAngle;
    private final Rotation2d kMaxAngle;

    private Rotation2d m_desiredAngle;

    public VariableHood(HoodIO io, ProfiledPIDController controller, Rotation2d minAngle, Rotation2d maxAngle,
                        double toleranceRad) {
        m_io = io;
        m_controller = controller;
        m_controller.setTolerance(toleranceRad);

        kMinAngle = minAngle;
        kMaxAngle = maxAngle;

        m_inputs = new HoodInputsAutoLogged();

        m_desiredAngle = Rotation2d.fromDegrees(0);

    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        m_controller.setGoal(m_desiredAngle.getRadians());
        double pidvoltage = m_controller.calculate(getAngle().getRadians());

        m_io.setVoltage(pidvoltage);

        Logger.getInstance().processInputs("VariableHood", m_inputs);
        Logger.getInstance().recordOutput("VariableHood/SetpointDegrees", getDesiredAngle().getDegrees());
        Logger.getInstance().recordOutput("VariableHood/AngleDegrees", getAngle().getDegrees());
        Logger.getInstance().recordOutput("VariableHood/OutputVoltage", pidvoltage);
    }

    public void setAngle(Rotation2d angle) {
        m_desiredAngle = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), kMinAngle.getRadians(), kMaxAngle.getRadians()));
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_inputs.angleRad);
    }

    public Rotation2d getDesiredAngle() {
        return m_desiredAngle;
    }

    public Command setAngleCommand(Rotation2d angle) {
        return runOnce(() -> setAngle(angle));
    }
}
