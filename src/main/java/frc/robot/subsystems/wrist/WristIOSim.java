package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.WristConstants;

public class WristIOSim implements WristIO {

    SingleJointedArmSim m_sim;

    double m_voltage;
    boolean m_brakeEnabled;

    public WristIOSim() {
        DCMotor gearbox = DCMotor.getNEO(1);
        double gearing = 1 / 0.03;
        double jKgMetersSquared = 1;
        double armLengthMeters = Units.inchesToMeters(7.85);
        double minAngleRads = WristConstants.kMinAngle.getRadians();
        double maxAngleRads = WristConstants.kMaxAngle.getRadians();
        boolean simulateGravity = false;

        m_sim = new SingleJointedArmSim(gearbox, gearing, jKgMetersSquared,
            armLengthMeters, minAngleRads, maxAngleRads, simulateGravity);
    }

    @Override
    public void updateInputs(WristInputs inputs) {
        m_sim.update(0.02);
        inputs.wristSpeed = getSpeed();
        inputs.brake = getBrakeMode();
        inputs.angleRad = getAngle().getRadians();
        inputs.currentAmps = m_sim.getCurrentDrawAmps();
        inputs.outputVoltage = getOutputVoltage();
    }

    @Override
    public void setVoltage(double voltage) {
        m_voltage = voltage;
        m_sim.setInputVoltage(voltage);
    }

    @Override
    public double getSpeed() {
        return m_sim.getVelocityRadPerSec();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_sim.getAngleRads());
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        m_brakeEnabled = enabled;
    }

    @Override
    public boolean getBrakeMode() {
        return m_brakeEnabled;
    }

    @Override
    public double getOutputVoltage() {
        return m_voltage;
    }
}
