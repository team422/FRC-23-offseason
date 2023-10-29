package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO m_io;
    public final IntakeInputsAutoLogged m_inputs;
    
    public int m_intakeFramesGamePiece;
    private final double kIntakeVoltage;
    private final double kIntakeHoldVoltage;
    private final double kIntakeOutFastVoltage;
    private final double kIntakeOutSlowVoltage;
    private double m_voltage;

    public Intake(IntakeIO io, double intakeVoltage, double intakeHoldVoltage) {
        m_io = io;
        m_inputs = new IntakeInputsAutoLogged();
        kIntakeVoltage = intakeVoltage;
        kIntakeHoldVoltage = -0.2;
        kIntakeOutFastVoltage = 12.0;
        kIntakeOutSlowVoltage = 6.0;
        m_voltage = 0;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        if (m_io.hasGamePiece() != null){
            if (m_io.hasGamePiece() == true) {
                m_intakeFramesGamePiece++;
                
            } else if (m_io.hasGamePiece() == false) {
                if(m_intakeFramesGamePiece > 0){
                m_intakeFramesGamePiece--;
                }
            }
        }
        if(m_intakeFramesGamePiece > 10){
            setVoltage(kIntakeHoldVoltage);
        } else{
            setVoltage(m_voltage);
        }
        Logger.getInstance().recordOutput("Intake/FramesWithObject", m_intakeFramesGamePiece);
        Logger.getInstance().processInputs("Intake", m_inputs);
        Logger.getInstance().recordOutput("Intake/Current", m_inputs.intakeOutputCurrent);
        Logger.getInstance().recordOutput("Intake/Speed", m_inputs.intakeSpeed);
        Logger.getInstance().recordOutput("Intake/Voltage", m_inputs.intakeOutputVoltage);
    }

    public void setVoltage(double voltage) {
        m_voltage = voltage;
        m_io.setVoltage(voltage);
    }

    public double getSpeed() {
        return m_inputs.intakeSpeed;
    }

    public boolean hasGamePiece() {
        return m_intakeFramesGamePiece > 10;
    }

    public Command setVoltageCommand(double voltage) {
        return runOnce(() -> this.setVoltage(voltage));
    }

    public Command setSpeedCommand(double speed) {
        return runOnce(() -> this.setVoltage(speed * 12));
    }

    public Command intakeAtVoltageCommand(double voltage) {
        return runEnd(
            () -> this.setVoltage(voltage),
            () -> this.setVoltage(0.0)
        );
    }


    public Command intakeCommand() {
        return intakeAtVoltageCommand(-kIntakeVoltage);
    }

    public Command outtakeSlowCommand() {
        return intakeAtVoltageCommand(kIntakeOutSlowVoltage);
    }

    public Command outtakeFastCommand() {
        return intakeAtVoltageCommand(kIntakeOutFastVoltage);
    }

    public Command holdCommand() {
        return intakeAtVoltageCommand(kIntakeHoldVoltage);
    }

    public Command stopCommand() {
        return runOnce(() -> setVoltage(0));
    }

}