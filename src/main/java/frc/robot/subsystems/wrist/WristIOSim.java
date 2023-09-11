package frc.robot.subsystems.wrist;

import java.util.Arrays;

import edu.wpi.first.math.system.plant.DCMotor;

public class WristIOSim implements WristIO{

    DCMotor[] motors = new DCMotor[20];
    double m_voltage;
    boolean m_brakeEnabled;

    public WristIOSim(){
        Arrays.fill(motors, DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(WristInputs inputs) {
        inputs.motorSpeed = getSpeeds();
        inputs.brakeEnabled = m_brakeEnabled;
        inputs.wristVoltage = m_voltage;
    }

    @Override
    public double[] getSpeeds() {
        double[] speeds = new double[motors.length];
        for (int i = 0; i < motors.length; i++) {
            speeds[i] = motors[i].getSpeed(40, m_voltage);
        }
        return speeds;
    }

    @Override
    public void setMotorSpeeds(double speed) {
        
    }

    @Override
    public void setMotorVoltages(double voltage) {
        m_voltage = voltage;       
    }

    @Override
    public void setMotorBrake(boolean brake) {
        m_brakeEnabled = brake;
    } 
}
