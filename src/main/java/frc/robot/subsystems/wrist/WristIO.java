package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.wrist.WristIO.WristInputs;

public interface WristIO extends LoggedIO<WristInputs>{
    @AutoLog
    public static class WristInputs {
        public double[] motorSpeed;
        public double wristVoltage;
        public boolean brakeEnabled;
        public double angleRad;
    }

    public double[] getSpeeds();

    public void setMotorSpeeds(double speed);

    public void setMotorVoltages(double Voltage);

    public void setMotorBrake(boolean brake);

}
