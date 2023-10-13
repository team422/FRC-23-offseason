package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EricNubControls;

public class OperatorControlsXbox implements OperatorControls {
    public CommandXboxController m_controller;
    public EricNubControls m_controls;
  
    public OperatorControlsXbox(int xboxControllerPort) {
        m_controller = new CommandXboxController(xboxControllerPort);
        m_controls = new EricNubControls();
    }

    @Override
    public Trigger wristManualUp() {
        return m_controller.povUp();
    }

    @Override
    public Trigger wristManualDown() {
        return m_controller.povDown();
    }
  
}
