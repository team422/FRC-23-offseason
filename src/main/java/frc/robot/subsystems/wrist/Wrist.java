package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.wrist.WristIO.WristInputs;

public class Wrist {
    private double kMinAngle;
    private double kMaxAngle;

    private WristIO m_IO;
    public WristInputsAutoLogged m_inputs;

    private PIDController m_controller;
    private Rotation2d desRotation2d;

    public Wrist(WristIO io, ProfiledPIDController wristController, Rotation2d minAngle, Rotation2d maxAngle) {
        m_IO = io;
        m_controller = new PIDController(0, 0, 0);
        m_inputs = new WristInputsAutoLogged();
        kMinAngle = minAngle.getRadians();
        kMaxAngle = maxAngle.getRadians();

    }

    public void periodic() {
        m_IO.updateInputs(m_inputs);
    }

    public void reset() {
        setAngle(Rotation2d.fromRadians(m_inputs.angleRad));
        m_controller.reset();
    }

    public void setAngle(Rotation2d angle) {
        desRotation2d = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), kMinAngle, kMaxAngle));
    }

    public void setBrakeMode(boolean brake) {
        m_IO.setMotorBrake(brake);
    }

    public void simulationPeriodic() {
    }

}
