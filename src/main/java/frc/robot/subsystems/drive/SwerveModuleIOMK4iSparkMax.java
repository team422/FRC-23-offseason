package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.lib.utils.CanSparkMaxSetup;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;

public class SwerveModuleIOMK4iSparkMax implements SwerveModuleIO {

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turningEncoder;

    private final SparkMaxPIDController m_turningController;
    private final SparkMaxPIDController m_driveController;

    private final CANCoder m_turningCANCoder;

    private double adjustedSpeed;

    public static class ModuleConstants {
        public static final double kDriveConversionFactor = 1 / 22.0409;
        public static final double kTurnPositionConversionFactor = 21.428;
        public static final TunableNumber kDriveP = Constants.ModuleConstants.kDriveP;
        public static final TunableNumber kDriveI = Constants.ModuleConstants.kDriveI;
        public static final TunableNumber kDriveD = Constants.ModuleConstants.kDriveD;
        public static final TunableNumber kTurningP = Constants.ModuleConstants.kTurningP;
        public static final TunableNumber kTurningI = Constants.ModuleConstants.kTurningI;
        public static final TunableNumber kTurningD = Constants.ModuleConstants.kTurningD;
        // public static final TunableNumber kDriveFF = RobotContainer.robotConstants.kDriveFF;
    }

    public SwerveModuleIOMK4iSparkMax(int driveMotorChannel, int turningMotorChannel, int turningCANCoderChannel) {
        CanSparkMaxSetup setup = new CanSparkMaxSetup();

        m_driveMotor = new CANSparkMax(driveMotorChannel, CANSparkMax.MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();
        setup.setupSparkMaxSlow(m_driveMotor);
        m_driveMotor.setInverted(true);
        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_driveMotor.enableVoltageCompensation(12);
        m_driveMotor.setSmartCurrentLimit(60);

        m_turningMotor = new CANSparkMax(turningMotorChannel, CANSparkMax.MotorType.kBrushless);
        m_turningMotor.restoreFactoryDefaults();
        setup.setupSparkMaxSlow(m_turningMotor);
        m_turningMotor.setInverted(true);
        m_turningMotor.setIdleMode(IdleMode.kBrake);
        m_turningMotor.setSmartCurrentLimit(60);
        m_turningMotor.enableVoltageCompensation(12);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveConversionFactor / 60.0);
        m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveConversionFactor);

        m_turningEncoder = m_turningMotor.getEncoder();
        m_turningEncoder.setVelocityConversionFactor((360.0 / ModuleConstants.kTurnPositionConversionFactor) / 60.0);
        m_turningEncoder.setPositionConversionFactor(360.0 / ModuleConstants.kTurnPositionConversionFactor);

        m_turningCANCoder = new CANCoder(turningCANCoderChannel);

        m_turningCANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        m_turningCANCoder.configSensorDirection(false);

        m_turningController = m_turningMotor.getPIDController();
        m_turningController.setPositionPIDWrappingEnabled(true);
        m_turningController.setPositionPIDWrappingMinInput(-180);
        m_turningController.setPositionPIDWrappingMaxInput(180);
    
        m_turningController.setP(ModuleConstants.kTurningP.get());
        m_turningController.setI(ModuleConstants.kTurningI.get());
        m_turningController.setD(ModuleConstants.kTurningD.get());

        m_driveController = m_driveMotor.getPIDController();
        m_driveController.setP(ModuleConstants.kDriveP.get());
        m_driveController.setI(ModuleConstants.kDriveI.get());
        m_driveController.setD(ModuleConstants.kDriveD.get());

    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveEncoder.getPosition(), getAngle());
    }

    public void resetDistance() {
        m_driveEncoder.setPosition(0.0);
    }

    @Override
    public void syncTurningEncoder() {
        m_turningEncoder.setPosition(getAbsoluteRotation().getDegrees());
    }

    @Override
    public void resetEncoders() {
        // Reset the cumulative rotation counts of the SparkMax motors
        m_turningEncoder.setPosition(0.0);

        m_turningCANCoder.setPosition(0.0);
        m_turningCANCoder.configMagnetOffset(
                m_turningCANCoder.configGetMagnetOffset() - m_turningCANCoder.getAbsolutePosition());
    }

    @Override
    public Rotation2d getAngle() {
        double angle = Units.degreesToRadians(m_turningEncoder.getPosition());
        return new Rotation2d(MathUtil.angleModulus(angle));
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        double driveOutput = state.speedMetersPerSecond;
        m_turningController.setReference(state.angle.getDegrees(), ControlType.kPosition);
        adjustedSpeed = driveOutput;
        m_driveController.setReference(driveOutput, ControlType.kVelocity, 0, adjustedSpeed);
    }

    public double deltaAdjustedAngle(double targetAngle, double currentAngle) {
        return ((targetAngle - currentAngle + 180) % 360 + 360) % 360 - 180;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(), getAngle());
    }

    @Override
    public SwerveModuleState getAbsoluteState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getAbsoluteRotation());
    }

    public double getDriveVelocityMetersPerSecond() {
        return m_driveEncoder.getVelocity();
    }

    public Rotation2d getAbsoluteRotation() {
        return Rotation2d.fromDegrees(m_turningEncoder.getPosition());
    }

    public CANSparkMax getTurnMotor() {
        return m_turningMotor;
      }
    
      public RelativeEncoder getTurnEncoder() {
        return m_turningEncoder;
      }
    
      public double getAbsoluteEncoder() {
        return m_turningCANCoder.getAbsolutePosition();
      }

      public double getDriveDistanceMeters() {
        return m_driveEncoder.getPosition();
      }

      @Override
      public void updateInputs(SwerveModuleInputs inputs) {
        inputs.driveDistanceMeters = getDriveDistanceMeters();
        inputs.driveVelocityMetersPerSecond = getDriveVelocityMetersPerSecond();
        inputs.turnAngleRad = getAngle().getRadians();
        inputs.turnRadsPerSecond = Units.degreesToRadians(m_driveEncoder.getVelocity());
        inputs.xDriveVelocityMetersPerSecond = Math.cos(inputs.turnAngleRad) * inputs.driveVelocityMetersPerSecond;
        inputs.yDriveVelocityMetersPerSecond = Math.sin(inputs.turnAngleRad) * inputs.driveVelocityMetersPerSecond;
      }
}
