package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.accelerometer.AccelerometerIO;
import frc.robot.subsystems.drive.accelerometer.AccelerometerInputsAutoLogged;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroInputsAutoLogged;

public class Drive extends SubsystemBase {

    private final SwerveModuleIO[] m_modules;
    public final SwerveModuleInputsAutoLogged[] m_inputs;

    private AccelerometerIO m_accel;
    public final AccelerometerInputsAutoLogged m_accelInputs;

    private final SwerveDrivePoseEstimator m_poseEstimator;

    private final GyroIO m_gyro;
    public final GyroInputsAutoLogged m_gyroInputs;

    private double m_simGyroLastUpdated;
    private boolean m_oneWheel = false;
    private int m_oneWheelIndex = 2;
    private final double[] m_lockAngles = new double[] { 45, 315, 45, 315 };

    /** Creates a new Drive. */
    public Drive(GyroIO gyro, AccelerometerIO accel, Pose2d startPose, SwerveModuleIO... modules) {
        m_modules = modules;
        m_gyro = gyro;
        m_gyroInputs = new GyroInputsAutoLogged();
        for (SwerveModuleIO module : m_modules) {
            module.resetDistance();
            module.syncTurningEncoder();
            // module.resetEncoders();
        }

        m_inputs = new SwerveModuleInputsAutoLogged[modules.length];
        for (int i = 0; i < modules.length; i++) {
            m_inputs[i] = new SwerveModuleInputsAutoLogged();
        }

        m_poseEstimator = new SwerveDrivePoseEstimator(Constants.DriveConstants.kDriveKinematics, m_gyro.getAngle(),
                getSwerveModulePositions(), startPose);

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
        SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
        for (int i = 0; i < m_modules.length; i++) {
            states[i] = m_modules[i].getState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run


        // Update Gyro Inputs/Logs
        m_gyro.updateInputs(m_gyroInputs);
        Logger.getInstance().processInputs("Gyro", m_gyroInputs);

        // Update Swerve Module Inputs/Logs
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].updateInputs(m_inputs[i]);
            Logger.getInstance().processInputs("Swerve Module " + i, m_inputs[i]);
        }

        Logger.getInstance().recordOutput("Robot/GyroPitchDrive",getGyro().getPitch().getDegrees());

        m_poseEstimator.update(m_gyro.getAngle(), getSwerveModulePositions());

        Logger.getInstance().recordOutput("Drive/Pose", getPose());
        Logger.getInstance().recordOutput("Drive/ModuleStates", getModuleStates());
        Logger.getInstance().recordOutput("Drive/ModuleAbsoluteStates", getModuleAbsoluteStates());
        

    }

    @Override
    public void simulationPeriodic() {
        double gyroDelta = getChassisSpeeds().omegaRadiansPerSecond;
        double ts = Timer.getFPGATimestamp();

        double deltaTime = ts - m_simGyroLastUpdated;

        m_gyro.addAngle(Rotation2d.fromRadians(gyroDelta * deltaTime));
        m_simGyroLastUpdated = ts;
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public SwerveModuleState[] getModuleAbsoluteStates() {
        SwerveModuleState[] positions = new SwerveModuleState[m_modules.length];
        for (int i = 0; i < m_modules.length; i++) {
          positions[i] = m_modules[i].getAbsoluteState();
        }
        return positions;
      }

    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(m_gyro.getAngle(), getSwerveModulePositions(), pose);
    }

    public void resetOdometry() {
        m_poseEstimator.resetPosition(m_gyro.getAngle(), getSwerveModulePositions(), new Pose2d());
    }

    public void brake(){
        drive(new ChassisSpeeds(0, 0, 0));
    }

    public void xBrake(){
        SwerveModuleState[] positions = new SwerveModuleState[m_modules.length];
        for (int i = 0; i < m_modules.length; i++) {
        m_modules[i].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(m_lockAngles[i])));
        }

    }

    public void drive(ChassisSpeeds speeds){
        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < states.length; i++) {
            states[i] = SwerveModuleState.optimize(states[i], m_modules[i].getAngle());
        }
        Logger.getInstance().recordOutput("Drive/DesiredModuleAbsoluteStates", states);

        setModuleStates(states);
    }

    public void setModuleStates(SwerveModuleState[] moduleStates){
        if (m_oneWheel){
            m_modules[m_oneWheelIndex].setDesiredState(moduleStates[m_oneWheelIndex]);
        }
        else{
            for (int i = 0; i < moduleStates.length; i++) {
                m_modules[i].setDesiredState(moduleStates[i]);
            }
        }
    }

    public GyroIO getGyro() {
        return m_gyro;
    }

    public AccelerometerIO getAccelerometerIO() {
        return m_accel;
    }

    public SwerveDrivePoseEstimator getPoseEstimator(){
        return m_poseEstimator;
    }

    public Command brakeCommand(){
        return runOnce(this::brake);
    }

    public Command manualFieldCentricCommand(){
        return runOnce(this::manualFieldCentric);
    }
    public void manualFieldCentric(){
        Pose2d oldPose = m_poseEstimator.getEstimatedPosition();
        m_poseEstimator.resetPosition(m_gyro.getAngle(), getSwerveModulePositions(),new Pose2d(oldPose.getX(),oldPose.getY(),new Rotation2d()) );
    }
}
