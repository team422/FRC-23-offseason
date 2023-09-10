package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import frc.lib.advantagekit.LoggedIO;

public interface IntakeIO extends LoggedIO<IntakeIO.IntakeInputs> {
    @AutoLog
    public static class IntakeInputs {
        public double motorSpeed;
        public boolean brake;
    }

    public void setVoltage(double voltage);

    public double getSpeed();

    public void brake(boolean enabled);

    public boolean getBrakeMode();
}
