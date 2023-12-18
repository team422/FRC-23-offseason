package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {
    private SingleJointedArmSim m_motorSim;
    
    private double m_voltage;

    public HoodIOSim() {
        DCMotor gearbox = DCMotor.getNEO(1);
        double gearing = 1 / 0.03;
        double jKgMetersSquared = 1;
        double armLengthMeters = Units.inchesToMeters(0);
        boolean simulateGravity = false;
        double minAngleRads = 0;
        double maxAngleRads = Units.degreesToRadians(90);
        m_motorSim = new SingleJointedArmSim(gearbox, gearing, jKgMetersSquared, armLengthMeters, minAngleRads, maxAngleRads, simulateGravity);
        m_voltage = 0;
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        m_motorSim.update(0.02);
        inputs.angleRad = getAngle().getRadians();
        inputs.outputVoltage = getOutputVoltage();
    }

    @Override
    public void setVoltage(double voltage) {
        m_voltage = voltage;
        m_motorSim.setInputVoltage(voltage);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_motorSim.getAngleRads());
    }

    @Override
    public double getOutputVoltage() {
        return m_voltage;
    }
}
