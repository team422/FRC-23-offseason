package frc.robot.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import frc.lib.advantagekit.LoggedIO;

public interface WristIO extends LoggedIO<WristIO.WristInputs> {
    @AutoLog
    public static class WristInputs {
        public Rotation2d angle;
        public double outputVoltage;
        public double currentAmps;
        public double velocity;
    }

    public void setVoltage(double voltage);

    public void setBrakeMode(boolean enabled);

    public Rotation2d getAngle();

    public boolean getBrakeMode();
}//hi guys