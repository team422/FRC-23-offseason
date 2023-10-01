package frc.robot.subsystems.drive.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.advantagekit.LoggedIO;

public interface GyroIO extends LoggedIO<GyroIO.GyroInputs> {
    @AutoLog
    public static class GyroInputs {
        public double angle;
        public double pitch;
    }

    public Rotation2d getAngle();

    public void addAngle(Rotation2d angle);

    public Rotation2d getPitch();
}
