package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControls {
    public double getDriveForward();

    public double getDriveLeft();
  
    public double getDriveRotation();

    public Trigger intakeButton();

    public Trigger outtakeButton();

    public Trigger wristButtonCube();

    public Trigger wristButtonShoot();

    public Trigger wristButtonStow();
}
