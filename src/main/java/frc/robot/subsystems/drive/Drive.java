package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.accelerometer.AccelerometerIO;
import frc.robot.subsystems.drive.accelerometer.AccelerometerInputsAutoLogged;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroInputsAutoLogged;

public class Drive extends SubsystemBase {

    private final SwerveModuleIO[] m_modules;
    private final SwerveModuleInputsAutoLogged[] m_inputs;

    // OLD SOK CODE
    // SOK vars
    // private final SecondOrderKinematics m_SecondOrderKinematics;
    // private SwerveModuleLateralAcceleration[] m_moduleAccelerations = new
    // SwerveModuleLateralAcceleration[] {
    // new SwerveModuleLateralAcceleration(), new SwerveModuleLateralAcceleration(),
    // new SwerveModuleLateralAcceleration(),
    // new SwerveModuleLateralAcceleration() };
    // private Rotation2d[] m_moduleSteerThetaVels = new Rotation2d[] { new
    // Rotation2d(), new Rotation2d(),
    // new Rotation2d(),
    // new Rotation2d() };
    // private Rotation2d[] m_moduleSteerOldTheta = new Rotation2d[] { new
    // Rotation2d(), new Rotation2d(),
    // new Rotation2d(),
    // new Rotation2d() };
    // private Rotation2d m_oldRobotTheta = new Rotation2d();
    // private Rotation2d m_robotThetaVel = new Rotation2d();
    // private final double m_deltaTime = 0.02;

    private AccelerometerIO m_accel;
    private final AccelerometerInputsAutoLogged m_accelInputs;

    private final SwerveDrivePoseEstimator m_poseEstimator;

    private final GyroIO m_gyro;
    private final GyroInputsAutoLogged m_gyroInputs;

    /** Creates a new Drive. */
    public Drive(GyroIO gyro, AccelerometerIO accel, Pose2d startPose, SwerveModuleIO... modules) {
        m_modules = modules;
        m_gyro = gyro;
        m_gyroInputs = new GyroInputsAutoLogged();
        for (SwerveModuleIO module : m_modules) {
            module.resetDistance();
            module.syncTurningEncoder();
        }

        m_inputs = new SwerveModuleInputsAutoLogged[modules.length];
        for (int i = 0; i < modules.length; i++) {
            m_inputs[i] = new SwerveModuleInputsAutoLogged();
        }

        m_poseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.kDriveKinematics, m_gyro.getAngle(),
                getSwerveModulePositions(), startPose);

        // OLD SOK CODE
        // m_SecondOrderKinematics = new SecondOrderKinematics();

        m_accel = accel;
        m_accelInputs = new AccelerometerInputsAutoLogged();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[m_modules.length];
        for (int i = 0; i < m_modules.length; i++) {
            positions[i] = m_modules[i].getPosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] positions = new SwerveModuleState[m_modules.length];
        for (int i = 0; i < m_modules.length; i++) {
            positions[i] = m_modules[i].getState();
        }
        return positions;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // NOTE TO FUTURE SOK CODERS: Fill in C.A. variables here, accel x/y can be obtained from acclerometer, and gyro velo/accel can be obtained from difference quotients or something idfk its 3 am

        // Update Gyro Inputs/Logs
        m_gyro.updateInputs(m_gyroInputs);
        Logger.getInstance().processInputs("Gyro", m_gyroInputs);

        // Update Swerve Module Inputs/Logs
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].updateInputs(m_inputs[i]);
            Logger.getInstance().processInputs("Swerve Module " + i, m_inputs[i]);
        }

        m_poseEstimator.update(m_gyro.getAngle(), getSwerveModulePositions());
        Logger.getInstance().recordOutput("Drive/ModuleStates", getModuleStates());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(m_gyro.getAngle(), getSwerveModulePositions(), pose);
    }

    public void resetOdometry() {
        m_poseEstimator.resetPosition(m_gyro.getAngle(), getSwerveModulePositions(), new Pose2d());
    }

    public void drive(ChassisSpeeds speeds){
        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveModuleState[] adjustedStates = new SwerveModuleState[states.length];
        for (int i = 0; i < states.length; i++) {
            adjustedStates[i] = new SwerveModuleState(states[i].speedMetersPerSecond, states[i].angle.minus(m_gyro.getAngle()));
        }
        setModuleStates(adjustedStates);
    }

    public void setModuleStates(SwerveModuleState[] moduleStates){
        for (int i = 0; i < moduleStates.length; i++) {
            m_modules[i].setDesiredState(moduleStates[i]);
        }
    }

    public GyroIO getGyroIO() {
        return m_gyro;
    }

    public AccelerometerIO getAccelerometerIO() {
        return m_accel;
    }

    public SwerveDrivePoseEstimator getPoseEstimator(){
        return m_poseEstimator;
    }
}
