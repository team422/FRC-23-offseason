package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorControls {
    public Trigger wristManualUp();

    public Trigger wristManualDown();

    public Trigger wristButtonShootLow();

    public Trigger outtakeSlowButton();

    public Trigger outtakeFastButton();
}
