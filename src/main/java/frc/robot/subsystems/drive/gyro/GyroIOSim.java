package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;

public class GyroIOSim implements GyroIO {
    
    AnalogGyroSim m_gyroSim;

    public GyroIOSim() {
        m_gyroSim = new AnalogGyroSim(1);
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.angle = m_gyroSim.getAngle();
        inputs.pitch = m_gyroSim.getAngle();        
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_gyroSim.getAngle());
    }

    @Override
    public void addAngle(Rotation2d angle) {
        m_gyroSim.setAngle(angle.getRadians() + angle.getDegrees());
    }

    @Override
    public Rotation2d getPitch() {
        return Rotation2d.fromRotations(m_gyroSim.getAngle());
    }

}
