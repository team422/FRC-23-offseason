package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeIOSim implements IntakeIO {

    DCMotor m_motorSim;
    double m_voltage;

    public IntakeIOSim() {
        m_motorSim = DCMotor.getNEO(1);
        m_voltage = 0;
    }

    @Override
    public double getSpeed(){
        return m_motorSim.getSpeed(450, m_voltage);
    }

    @Override
    public void setDesiredVoltage(double voltage){
        m_voltage = voltage;
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.motorSpeed = m_motorSim.getSpeed(450, m_voltage);      
    }

}
