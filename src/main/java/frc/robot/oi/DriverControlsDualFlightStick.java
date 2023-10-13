package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverControlsDualFlightStick implements DriverControls {
    public CommandJoystick m_leftJoystick;
    public CommandJoystick m_rightJoystick;

    public DriverControlsDualFlightStick(int leftJoystick, int rightJoystick) {
        m_leftJoystick = new CommandJoystick(leftJoystick);
        m_rightJoystick = new CommandJoystick(rightJoystick);
    }

    @Override
    public double getDriveX() {
        return -Math.signum(m_leftJoystick.getY()) * Math.pow(m_leftJoystick.getY(), 2);
    }

    @Override
    public double getDriveY() {
        return -Math.signum(m_leftJoystick.getX()) * Math.pow(m_leftJoystick.getX(), 2);
    }

    @Override
    public double getDriveRotation() {
        return -Math.signum(m_rightJoystick.getX()) * Math.pow(m_rightJoystick.getX(), 2);
    }

    @Override
    public Trigger intakeButton() {
        return m_leftJoystick.button(1);
    }

    @Override
    public Trigger outtakeButton() {
        return m_rightJoystick.button(1);
    }

    @Override
    public Trigger wristButtonCube() {
        return m_leftJoystick.button(3);
    }

    @Override
    public Trigger wristButtonShoot() {
        return m_rightJoystick.button(3);
    }

    @Override
    public Trigger wristButtonStow() {
        return m_leftJoystick.button(2);
    }

    @Override
    public Trigger wristManualUp() {
        return m_leftJoystick.button(8);
    }

    @Override
    public Trigger wristManualDown() {
        return m_leftJoystick.button(9);
    }
}
