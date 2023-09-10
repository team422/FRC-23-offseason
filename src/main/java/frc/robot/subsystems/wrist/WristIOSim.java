package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.WristConstants;

public class WristIOSim implements WristIO {

    SingleJointedArmSim m_sim;

    DCMotor[] m_motorsSim;
    double m_voltage;
    boolean m_brakeEnabled;

    public WristIOSim() {
        // m_sim = new SingleJointedArmSim(DCMotor.getNEO(WristConstants.kNumSimMotors),
        // WristConstants.kGearRatio, 0.01, Units.inchesToMeters(7.85),
        // WristConstants.kMinAngle.getRadians(), WristConstants.kMaxAngle.getRadians(), false);
        
        System.out.println("here");
        DCMotor gearbox = DCMotor.getNEO(WristConstants.kNumSimMotors);
        double gearing = WristConstants.kGearRatio;
        double jKgMetersSquared = 0.01;
        double armLengthMeters = Units.inchesToMeters(7.85);
        double minAngleRads = WristConstants.kMinAngle.getRadians();
        double maxAngleRads = WristConstants.kMaxAngle.getRadians();
        boolean simulateGravity = false;

        System.out.println("here2");
        m_sim = new SingleJointedArmSim(gearbox, gearing, jKgMetersSquared,
            armLengthMeters, minAngleRads, maxAngleRads, simulateGravity);
        System.out.println("here?");

        // m_motorsSim = new DCMotor[WristConstants.kNumSimMotors];
        // Arrays.fill(m_motorsSim, DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(WristInputs inputs) {
        inputs.motorSpeed = getSpeed();
        inputs.brake = getBrakeMode();
        inputs.angleRad = getAngle().getRadians();
    }

    @Override
    public void setVoltage(double voltage) {
        m_sim.setInputVoltage(-voltage / 2);
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
}
