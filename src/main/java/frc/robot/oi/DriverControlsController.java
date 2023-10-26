package frc.robot.oi;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.EricNubControls;

public class DriverControlsController implements DriverControls {
    CommandXboxController m_controller;
    EricNubControls m_controls;

    public DriverControlsController(int controller_port) {
        m_controller = new CommandXboxController(controller_port);
        m_controls = new EricNubControls();
    }

    @Override
    public double getDriveY() {
        return m_controller.getLeftX();
    }

    @Override
    public double getDriveX() {
        double val = m_controls.addDeadzoneScaled(m_controller.getLeftY(), 0.03);
        return -Math.signum(val) * Math.pow(val, 2);
    }

    @Override
    public double getDriveRotation() {
        double val = m_controls.addDeadzoneScaled(m_controller.getRightX(), 0.03);
        return Math.signum(val) * Math.pow(val, 2);
        
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
    public Trigger manualFieldReset() {
        // TODO Auto-generated method stub
        return m_controller.rightStick();
    }

    @Override
    public Trigger balance() {
        // TODO Auto-generated method stub
        return m_controller.leftStick();
    }
    public Trigger resetWristEncoder() {
        return m_controller.povUp();
    }
    
}
