package frc.robot.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import frc.robot.wrist.WristIO.WristInputs;

public class Wrist extends SubsystemBase {
    private WristIO m_io;
    private ProfiledPIDController m_controller;
    private Rotation2d kMinAngle;
    private Rotation2d kMaxAngle;
    private Rotation2d m_desiredAngle;
    private WristInputs m_inputs;

    public Wrist(WristIO io, ProfiledPIDController controller, Rotation2d minAngle, Rotation2d maxAngle) {
        m_controller = controller;
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
    
    public Rotation2d getAngle() {
        return m_inputs.angle;
    }
}