package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private FlywheelIO m_io;
    private ProfiledPIDController m_controller;
    public FlywheelInputsAutoLogged m_inputs;

    public Flywheel(FlywheelIO io, ProfiledPIDController controller, double toleranceMetersPerSecond) {
        m_io = io;
        m_controller = controller;
        m_controller.setTolerance(toleranceMetersPerSecond);
        m_inputs = new FlywheelInputsAutoLogged();
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        double pidVoltage = m_controller.calculate(m_inputs.velocityMetersPerSecond, m_controller.getGoal());
        m_io.setVoltage(pidVoltage);

        Logger.getInstance().processInputs("Flywheel Inputs", m_inputs);

        Logger.getInstance().recordOutput("Flywheel/VelocityMetersPerSecond", getVelocityMetersPerSecond());
        Logger.getInstance().recordOutput("Flywheel/VelocityDegreesPerSecond", Units.radiansToDegrees(getVelocity()));
        Logger.getInstance().recordOutput("Flywheel/SetpointMetersPerSecond", getDesiredVelocity());
        Logger.getInstance().recordOutput("Flywheel/OutputVoltage", pidVoltage);
        Logger.getInstance().recordOutput("Flywheel/HasGamepiece", m_io.getHasGamepiece());
    }

    public void setDesiredVelocity(double velocityMetersPerSecond) {
        m_controller.setGoal(velocityMetersPerSecond);
    }

    public double getDesiredVelocity() {
        return m_controller.getGoal().position;
    }

    public void setDesiredVelocityRPM(double velocityRPM) {
        // this is a big boi but he just converts rpm to m/s
        setDesiredVelocity(2 * Math.PI * velocityRPM / 60 * m_io.getFlywheelLengthMeters());
    }

    public double getVelocityMetersPerSecond() {
        return m_io.getVelocityMetersPerSecond();
    }

    public double getVelocity() {
        return m_io.getVelocityRadPerSecond();
    }

    public boolean withinTolerance() {
        return m_controller.atGoal();
    }

    public Command setDesiredVelocityCommand(double velocityMetersPerSecond) {
        return runOnce(() -> setDesiredVelocity(velocityMetersPerSecond));
    }

    public Command toggleGamepieceCommand() {
        return runOnce(m_io::toggleHasGamePiece);
    }
}
