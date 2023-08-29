package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class DriverControlsDualFlightStick implements DriverControls {
    public CommandJoystick m_leftJoystick;
    public CommandJoystick m_rightJoystick;
  
    public DriverControlsDualFlightStick(int leftJoystick, int rightJoystick) {
      m_leftJoystick = new CommandJoystick(leftJoystick);
      m_rightJoystick = new CommandJoystick(rightJoystick);
    }
}
