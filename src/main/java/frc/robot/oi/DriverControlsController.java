package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsController implements DriverControls {
    CommandXboxController m_controller;

    public DriverControlsController(int controller_port) {
        m_controller = new CommandXboxController(controller_port);
    }

    @Override
    public double getDriveY() {
        return m_controller.getLeftX();
    }

    @Override
    public double getDriveX() {
        return m_controller.getLeftY();
    }

    @Override
    public double getDriveRotation() {
        return m_controller.getRightX();
    }

    @Override
    public Trigger intakeButton() {
        return m_controller.rightTrigger(0.1);
    }

    @Override
    public Trigger outtakeButton() {
        // TODO Auto-generated method stub
        return m_controller.leftTrigger(0.1);
    }

    @Override
    public Trigger wristButtonIntake() {
        // TODO Auto-generated method stub
        return m_controller.a();
    }

    @Override
    public Trigger wristButtonShoot() {
        // TODO Auto-generated method stub
        return  m_controller.rightBumper();
    }

    @Override
    public Trigger wristButtonStow() {
        // TODO Auto-generated method stub
        return new Trigger();
    }

    @Override
    public Trigger wristManualUp() {
        // TODO Auto-generated method stub
        return new Trigger();
    }

    @Override
    public Trigger wristManualDown() {
        // TODO Auto-generated method stub
        return new Trigger();
    }

    @Override
    public Trigger resetWristEncoder() {
        return m_controller.povUp();
    }
    
}
