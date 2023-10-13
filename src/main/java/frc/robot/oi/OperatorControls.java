package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorControls {
    // ash insisted that i make these for operatorcontrols
    // dont be surprised if they get removed soon
    public Trigger wristManualUp();

    public Trigger wristManualDown();
}
