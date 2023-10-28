package frc.robot.subsystems.wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;

public class WristIOCANSparkMax implements WristIO {
    private CANSparkMax m_wristLeader;
    private CANSparkMax m_wristFollower;
    private SparkMaxAbsoluteEncoder m_encoder;
    private RelativeEncoder m_relativeEncoder;
    // private RelativeEncoder m_encoder;
    private boolean m_brakeModeEnabled;
    private double kBack;

    public WristIOCANSparkMax(int leaderPort, int followerPort, double encoderOffset, double gearRatio) {
        double encoderOffsetVal = 143-52-15;
        double absoluteEncoderOffset = 180;
        kBack = 58;
        m_wristLeader = new CANSparkMax(leaderPort, CANSparkMax.MotorType.kBrushless);
        m_wristFollower = new CANSparkMax(followerPort, CANSparkMax.MotorType.kBrushless);
        m_wristFollower.follow(m_wristLeader);
        m_encoder = m_wristLeader.getAbsoluteEncoder(Type.kDutyCycle);
        m_wristLeader.setInverted(false);
        m_wristLeader.setIdleMode(IdleMode.kBrake);
        m_brakeModeEnabled = true;
        m_encoder.setPositionConversionFactor(gearRatio * (1/0.625)); 
        m_encoder.setInverted(true);
        m_encoder.setZeroOffset(encoderOffsetVal);
        m_relativeEncoder = m_wristLeader.getEncoder();
        // m_relativeEncoder.setInverted(true);
        m_relativeEncoder.setPositionConversionFactor(15.714);
        syncRelativeAndAbsolute(m_encoder.getPosition(),absoluteEncoderOffset);
    }

    public void syncRelativeAndAbsolute(double val,double absoluteEncoderOffset){
        m_relativeEncoder.setPosition(val - absoluteEncoderOffset);
    }

    @Override
    public void updateInputs(WristInputs inputs) {
        inputs.angleRad = getAngle().getRadians();
        inputs.angleDegreeRelative = getAngle().getDegrees();
        inputs.angleDegreeAbsolute = Rotation2d.fromDegrees(m_encoder.getPosition()).getDegrees();
        inputs.outputVoltage = getOutputVoltage();
        inputs.currentAmps = m_wristLeader.getOutputCurrent();
        inputs.currentAmpsSecondary = m_wristFollower.getOutputCurrent();
        inputs.wristSpeed = getSpeed();
        // System.out.println(m_encoder.getZeroOffset());
    }

    @Override
    public void setVoltage(double voltage) {
        m_wristLeader.setVoltage(voltage);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        if (enabled){
            m_wristLeader.setIdleMode(IdleMode.kBrake);
            m_brakeModeEnabled = true;
        }
        else {
            // m_wristLeader.setIdleMode(IdleMode.kCoast);
            m_brakeModeEnabled = false;

        }
    }
    
    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(m_encoder.getPosition()).minus(Rotation2d.fromDegrees(180));
    }

    @Override
    public boolean getBrakeMode() {
        return m_brakeModeEnabled;
    }

    @Override
    public double getSpeed() {
        return m_encoder.getVelocity();
    }

    @Override
    public double getOutputVoltage() {
        return m_wristLeader.getAppliedOutput() * m_wristLeader.getBusVoltage();
    }

    @Override
    public void setEncoderOffset(double offset) {
        m_encoder.setZeroOffset(offset);
    }

    @Override
    public void localizeEncoder() {
        m_relativeEncoder.setPosition(kBack);
    }
        
}