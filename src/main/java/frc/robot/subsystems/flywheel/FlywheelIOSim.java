package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
    private FlywheelSim m_sim;
    private final double m_flywheelLengthMeters;
    private boolean m_hasGamepiece;
    private double m_gamepieceMultiplier; // for simulating speed when game piece

    public FlywheelIOSim() {
        DCMotor gearbox = DCMotor.getFalcon500(1);
        double gearing = 3;
        double jKgMetersSquared = 1;
        m_sim = new FlywheelSim(gearbox, gearing, jKgMetersSquared);
        m_flywheelLengthMeters = Units.inchesToMeters(9);
        m_hasGamepiece = false;
        m_gamepieceMultiplier = 0.5;
    }

    @Override
    public void updateInputs(FlywheelInputs inputs) {
        m_sim.update(0.02);
        inputs.velocityMetersPerSecond = getVelocityMetersPerSecond();
        inputs.velocityRadiansPerSecond = getVelocityRadPerSecond();
    }

    @Override
    public void setVoltage(double volts) {
        m_sim.setInputVoltage(volts * (m_hasGamepiece ? m_gamepieceMultiplier : 1));
    }

    @Override
    public double getVelocityMetersPerSecond() {
        return getVelocityRadPerSecond() * m_flywheelLengthMeters;
    }

    @Override
    public double getVelocityRadPerSecond() {
        
        return m_sim.getAngularVelocityRadPerSec();
    }

    @Override
    public double getFlywheelLengthMeters() {
        return m_flywheelLengthMeters;
    }

    @Override
    public void toggleHasGamePiece() {
        m_hasGamepiece = !m_hasGamepiece;
    }

    @Override
    public boolean getHasGamepiece() {
        return m_hasGamepiece;
    }
}
