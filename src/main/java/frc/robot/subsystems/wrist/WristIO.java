package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;
import frc.lib.advantagekit.LoggedIO;
import edu.wpi.first.math.geometry.Rotation2d;

public interface WristIO extends LoggedIO<WristIO.WristInputs> {
    @AutoLog
    public static class WristInputs {
        public double motorSpeed;
        public boolean brake;
        public double angleRad;
    }

    public void setVoltage(double voltage);

    public double getSpeed();

    public Rotation2d getAngle();

    public void setBrakeMode(boolean enabled);

    public boolean getBrakeMode();
}
