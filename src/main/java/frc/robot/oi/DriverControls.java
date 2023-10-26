package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
    public double getDriveY();

    public double getDriveX();
  
    public double getDriveRotation();

    public Trigger intakeButton();

    public Trigger outtakeButton();

    public Trigger wristButtonIntake();

    public Trigger wristButtonShoot();

    public Trigger wristButtonStow();

    public Trigger wristManualUp();

    public Trigger wristManualDown();

    public Trigger manualFieldReset();

    public Trigger balance();
    public Trigger resetWristEncoder();
}
