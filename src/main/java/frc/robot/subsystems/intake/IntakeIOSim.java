package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeIOSim implements IntakeIO {

    DCMotor m_motorSim;
    double m_voltage;

    public IntakeIOSim() {
        
    }

    @Override
    public void setMotorSpeed(double speed){
        m_motorSim = DCMotor.getNEO(0);
        m_voltage = 0;
    }

    @Override
    public double getSpeed(){
        return m_motorSim.getSpeed(40, m_voltage);
    }

    @Override
    public void setIntakeVoltage(double Voltage){
        m_voltage = Voltage;
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.motorSpeed = m_motorSim.getSpeed(450, m_voltage);      
    }

}
