package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;

public class Wrist extends SubsystemBase {
    private final WristIO m_io;
    public final WristInputsAutoLogged m_inputs;

    private final ProfiledPIDController m_controller;
    private final ArmFeedforward m_feedforward;
    private final Rotation2d kMinAngle;
    private final Rotation2d kMaxAngle;
    private Rotation2d m_desiredAngle;

    private double m_lastTime;
    private double m_lastVelocitySetpoint;

    public Wrist(WristIO io, ProfiledPIDController controller, ArmFeedforward feedforward, Rotation2d minAngle, Rotation2d maxAngle,
            double toleranceRad) {
        m_controller = controller;
        m_controller.setTolerance(toleranceRad);
        m_io = io;
        m_feedforward = feedforward;
        kMaxAngle = maxAngle;
        kMinAngle = minAngle;
        m_desiredAngle = Rotation2d.fromDegrees(0);
        m_inputs = new WristInputsAutoLogged();
    }
    
    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        double dt = Timer.getFPGATimestamp() - m_lastTime;

        double currAngle = m_inputs.angleRad;
        double pidVoltage = m_controller.calculate(currAngle);

        double positionSetpoint = m_controller.getGoal().position;
        double velocitySetpoint = m_controller.getGoal().velocity;
        double accelerationSetpoint = (velocitySetpoint - m_lastVelocitySetpoint) / dt;
        double feedforwardVoltage = m_feedforward.calculate(positionSetpoint, velocitySetpoint, accelerationSetpoint);

        double outputVoltage = pidVoltage + feedforwardVoltage;
        if (Robot.isSimulation()) {
            m_io.setVoltage(pidVoltage);
        } else {
            m_io.setVoltage(outputVoltage);
        }
        
        Logger.getInstance().processInputs("Wrist", m_inputs);
        Logger.getInstance().recordOutput("Wrist/SetpointDegrees", Units.radiansToDegrees(m_controller.getGoal().position));
        Logger.getInstance().recordOutput("Wrist/CurrentDegrees", Units.degreesToRadians(m_inputs.angleRad));

        m_lastTime = Timer.getFPGATimestamp();
        m_lastVelocitySetpoint = velocitySetpoint;
    }

    public void setAngle(Rotation2d angle) {
        m_desiredAngle = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), kMinAngle.getRadians(), kMaxAngle.getRadians()));
        m_controller.setGoal(m_desiredAngle.getRadians());
    }

    public void setBrakeMode(boolean enabled) {
        m_io.setBrakeMode(enabled);
    }
    
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_inputs.angleRad);
    }

    public boolean withinTolerance() {
        return m_controller.atGoal();
    }

    public Command setAngleCommand(Rotation2d angle) {
        return run(() -> setAngle(angle)).until(this::withinTolerance);
    }

}
