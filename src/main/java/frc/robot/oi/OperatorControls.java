package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorControls {

    public Trigger wristManualUp();

    public Trigger wristManualDown();

    public Trigger outtakeFastButton();

    public Trigger outtakeSlowButton();

    public Trigger wristButtonShootLow();

    public Trigger hoodStow();

    public Trigger hoodLow();

    public Trigger hoodHigh();
}
