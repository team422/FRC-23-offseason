package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;
import frc.lib.advantagekit.LoggedIO;
import edu.wpi.first.math.geometry.Rotation2d;

public interface WristIO extends LoggedIO<WristIO.WristInputs> {
    @AutoLog
    public static class WristInputs {
        public Rotation2d angle;
        public double outputVoltage;
        public double currentAmps;
        public double wristSpeed;
        public boolean brake;
    }

    public void setVoltage(double voltage);

    public void setBrakeMode(boolean enabled);

    public double getSpeed();

    public Rotation2d getAngle();

    public boolean getBrakeMode();
}
