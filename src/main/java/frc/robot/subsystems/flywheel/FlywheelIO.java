package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;

public interface FlywheelIO extends LoggedIO<FlywheelIO.FlywheelInputs> {
    @AutoLog
    public static class FlywheelInputs {
        double velocityMetersPerSecond;
        double velocityRadiansPerSecond;
    }

    public void setVoltage(double volts);

    public double getVelocityMetersPerSecond();

    public double getVelocityRadPerSecond();

    public double getFlywheelLengthMeters();

    public default void toggleHasGamePiece() {}; // for sim only

    public default boolean getHasGamepiece() { return false; }; // for sim only
}
