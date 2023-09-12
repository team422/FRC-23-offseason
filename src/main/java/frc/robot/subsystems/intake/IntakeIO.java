package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.intake.IntakeIO.IntakeInputs;

public interface IntakeIO extends LoggedIO<IntakeInputs> {

    @AutoLog
    public static class IntakeInputs {
        public double motorSpeed;
    }

    public double getSpeed();

    public void setDesiredVoltage(double Voltage);

    public void updateInputs(IntakeInputs inputs);

}
