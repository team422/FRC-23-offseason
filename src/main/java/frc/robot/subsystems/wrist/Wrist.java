package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;

public class Wrist extends SubsystemBase {
    private final WristIO m_io;
    public final WristInputsAutoLogged m_inputs;

    private ProfiledPIDController m_controller;
    private ArmFeedforward m_feedforward;
    private final Rotation2d kMinAngle;
    private final Rotation2d kMaxAngle;
    private Rotation2d m_desiredAngle;

    private double m_lastTime;
    private double m_lastVelocitySetpoint;
    private boolean m_kZeroing;
    private final double kManualMoveRad;

    public Wrist(WristIO io, ProfiledPIDController controller, ArmFeedforward feedforward, Rotation2d minAngle, Rotation2d maxAngle,
            double toleranceRad, double manualMoveRad) {
        m_io = io;
        m_controller = controller;
        m_controller.setTolerance(toleranceRad);
        
        m_feedforward = feedforward;
        
        kMaxAngle = maxAngle;
        kMinAngle = minAngle;
        m_desiredAngle = Rotation2d.fromDegrees(20);
        m_kZeroing = false;
        m_inputs = new WristInputsAutoLogged();

        kManualMoveRad = manualMoveRad;
    }
    
    @Override
    public void periodic() {
        if(WristConstants.kP.hasChanged() || WristConstants.kI.hasChanged() || WristConstants.kD.hasChanged()){
            m_controller = new ProfiledPIDController(WristConstants.kP.get(), WristConstants.kI.get(), WristConstants.kD.get(), new Constraints(WristConstants.kWristVelo.get(), WristConstants.kWristAccel.get()));
        }
        if (WristConstants.kWristkg.hasChanged() || WristConstants.kWristkv.hasChanged() || WristConstants.kWristks.hasChanged() ){
            m_feedforward = new ArmFeedforward(WristConstants.kWristks.get(),WristConstants.kWristkg.get(), WristConstants.kWristkv.get());
        }
        m_io.updateInputs(m_inputs);

        double dt = Timer.getFPGATimestamp() - m_lastTime;

        double currAngle = m_inputs.angleRad;

        double pidVoltage = m_controller.calculate(currAngle,m_desiredAngle.getRadians());

        double positionSetpoint = Rotation2d.fromRadians(m_inputs.angleRad).plus(Rotation2d.fromDegrees(-90)).getRadians();
        double velocitySetpoint = m_inputs.wristSpeed;
        double accelerationSetpoint = (velocitySetpoint - m_lastVelocitySetpoint) / dt;
        // double feedforwardVoltage = m_feedforward.calculate(positionSetpoint, velocitySetpoint, accelerationSetpoint);
        double feedforwardVoltage = m_feedforward.calculate(positionSetpoint, velocitySetpoint);

        double outputVoltage = pidVoltage + feedforwardVoltage;
        if (!m_kZeroing){
            if (Robot.isSimulation()) {
                setVoltage(pidVoltage);
            } else {
                setVoltage(outputVoltage);
            }
        } else {
            setVoltage(4);
            m_io.localizeEncoder();
        }
        
        Logger.getInstance().processInputs("Wrist", m_inputs);
        Logger.getInstance().recordOutput("Wrist/SetpointDegrees",m_desiredAngle.getDegrees());
        Logger.getInstance().recordOutput("Wrist/CurrentDegrees", Units.radiansToDegrees(m_inputs.angleRad));
        Logger.getInstance().recordOutput("Wrist/FFOutputVoltage", feedforwardVoltage);
        Logger.getInstance().recordOutput("Wrist/PIDOutputVoltage", pidVoltage);
        Logger.getInstance().recordOutput("Wrist/angle", m_io.getAngle().getDegrees());
        Logger.getInstance().recordOutput("Wrist/ZeroingEnabled", m_kZeroing);
        m_lastTime = Timer.getFPGATimestamp();
        m_lastVelocitySetpoint = velocitySetpoint;
    }

    public void setAngle(Rotation2d angle) {
        m_desiredAngle = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), kMinAngle.getRadians(), kMaxAngle.getRadians()));
        m_controller.setGoal(m_desiredAngle.getRadians());
    }

    public void setVoltage(double voltage) {
        m_io.setVoltage(voltage);
    }

    public void setVoltageToNotBreak(double voltage) {
        if (m_inputs.angleRad < kMinAngle.getRadians() && voltage < 0) {
            voltage = 0;
        } else if (m_inputs.angleRad > kMaxAngle.getRadians() && voltage > 0) {
            voltage = 0;
        }
        setVoltage(voltage);
    }

    public void setBrakeMode(boolean enabled) {
        m_io.setBrakeMode(enabled);
    }
    
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_inputs.angleRad);
    }

    public boolean withinTolerance() {
        return m_controller.atGoal();
    }

    public Command setAngleCommand(Rotation2d angle) {
        return runOnce(() -> setAngle(angle));
    }

    public Command manualUpCommand() {
        return runEnd(() -> {
            setAngle(Rotation2d.fromRadians(m_inputs.angleRad + kManualMoveRad));
        }, () -> setVoltage(0));
    }

    public Command manualDownCommand() {
        return runEnd(() -> {
            setAngle(Rotation2d.fromRadians(m_inputs.angleRad - kManualMoveRad));
        }, () -> setVoltage(0));
    }

    public void resetEncoderOffset() {
        m_io.setEncoderOffset(0.0);
    }

    public Command resetEncoderCommand() {
        return runOnce(() -> resetEncoderOffset()).andThen(() -> m_io.setEncoderOffset(m_inputs.angleRad + Units.degreesToRadians(2)));
    }

    public Command moveAndResetEncoder() {
        return runEnd(()->{m_kZeroing = true;}, ()->{m_kZeroing=false;});
    }

}
