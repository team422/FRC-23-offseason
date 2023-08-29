package frc.robot.util;

import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SecondOrderKinematics {

    private final SimpleMatrix m_inverseKinematics;
    private final SimpleMatrix m_forwardKinematics;

    private final int m_numModules;
    private final Translation2d[] m_modules;
    private SwerveModuleState[] m_moduleStates;
    private SwerveModuleAcceleration[] m_moduleAccelerations;
    private Translation2d m_prevCoR = new Translation2d();

    public SecondOrderKinematics(Translation2d... wheelsMeters) {
        if (wheelsMeters.length < 2) {
            throw new IllegalArgumentException("A swerve drive requires at least two modules");
        }
        m_numModules = wheelsMeters.length;
        m_modules = Arrays.copyOf(wheelsMeters, m_numModules);
        m_moduleStates = new SwerveModuleState[m_numModules];
        Arrays.fill(m_moduleStates, new SwerveModuleState());
        m_inverseKinematics = new SimpleMatrix(m_numModules * 2, 4);

        // Inverse Kinematics Matrix used to find lateral acclerations for each module
        for (int i = 0; i < m_numModules; i++) {
            m_inverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -m_modules[i].getX(), -m_modules[i].getY());
            m_inverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, -m_modules[i].getY(), +m_modules[i].getX());
        }

        m_forwardKinematics = m_inverseKinematics.pseudoInverse();

    }

    public SwerveModuleAcceleration[] toSwerveModuleAccelerations(
            ChassisAccelerations chassisAccelerations, Translation2d centerOfRotationMeters) {
        if (chassisAccelerations.axMetersPerSecondSquared == 0 && chassisAccelerations.ayMetersPerSecondSquared == 0
                && chassisAccelerations.alphaRadiansPerSecondSquared == 0
                && chassisAccelerations.omegaRadiansPerSecond == 0) {
            SwerveModuleAcceleration[] newAccelerations = new SwerveModuleAcceleration[m_numModules];
            for (int i = 0; i < m_numModules; i++) {
                newAccelerations[i] = new SwerveModuleAcceleration(0.0, new Rotation2d());
            }

            m_moduleAccelerations = newAccelerations;
            return m_moduleAccelerations;
        }

        if (!centerOfRotationMeters.equals(m_prevCoR)) {
            for (int i = 0; i < m_numModules; i++) {
                m_inverseKinematics.setRow(i * 2 + 0, 0, 1, 0, -m_modules[i].getX() + centerOfRotationMeters.getX(),
                        -m_modules[i].getY() + centerOfRotationMeters.getY());
                m_inverseKinematics.setRow(i * 2 + 1, 0, 0, 1, -m_modules[i].getY() + centerOfRotationMeters.getY(),
                        +m_modules[i].getX() + centerOfRotationMeters.getX());
            }
            m_prevCoR = centerOfRotationMeters;
        }

        SimpleMatrix chassisAccelerationVector = new SimpleMatrix(4, 1);
        chassisAccelerationVector.setColumn(
            0, 
            0, 
            chassisAccelerations.axMetersPerSecondSquared,
            chassisAccelerations.ayMetersPerSecondSquared, 
            chassisAccelerations.omegaRadiansPerSecond, 
            chassisAccelerations.alphaRadiansPerSecondSquared);
        
        SimpleMatrix moduleAccelertionMatrix = m_forwardKinematics.mult(chassisAccelerationVector);

        m_moduleAccelerations = new SwerveModuleAcceleration[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            double x = moduleAccelertionMatrix.get(i * 2 + 0, 0);
            double y = moduleAccelertionMatrix.get(i * 2 + 1, 0);

            double speed = Math.hypot(x, y);
            Rotation2d angle = new Rotation2d(x, y);

            m_moduleAccelerations[i] = new SwerveModuleAcceleration(speed, angle);
        }

        return m_moduleAccelerations;
    }

}
