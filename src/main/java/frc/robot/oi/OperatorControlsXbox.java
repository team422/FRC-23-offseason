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

    @Override
    public Trigger outtakeSlowButton() {
        return m_controller.leftTrigger();
    }

    @Override
    public Trigger outtakeFastButton() {
        return m_controller.rightTrigger();
    }

    @Override
    public Trigger wristButtonShootLow() {
        return m_controller.a();
    }

    @Override
    public Trigger hoodStow() {
        return m_controller.b();
    }

    @Override
    public Trigger hoodLow() {
        return m_controller.x();
    }

    @Override
    public Trigger hoodHigh() {
        return m_controller.y();
    }

    @Override
    public Trigger flywheelStart() {
        return m_controller.rightBumper();
    }

    @Override
    public Trigger flywheelStop() {
        return m_controller.leftBumper();
    }
    
}
