package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.advantagekit.LoggedIO;

public interface HoodIO extends LoggedIO<HoodIO.HoodInputs> {
    @AutoLog
    public static class HoodInputs {
        public double angleRad;
        public double outputVoltage;
    }

    public void setVoltage(double voltage);

    public Rotation2d getAngle();

    public double getOutputVoltage();
}
