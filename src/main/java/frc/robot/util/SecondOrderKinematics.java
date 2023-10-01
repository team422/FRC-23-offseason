package frc.robot.util;

import java.util.Arrays;
import java.util.Collections;

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

    /**
     * Performs inverse kinematics to return the module accelerations from a desired
     * chassis acceleration.
     *
     * <p>
     * This function also supports variable centers of rotation. During normal
     * operations, the
     * center of rotation is usually the same as the physical center of the robot;
     * therefore, the
     * argument is defaulted to that use case. However, if you wish to change the
     * center of rotation
     * for evasive maneuvers, vision alignment, or for any other use case, you can
     * do so.
     *
     * <p>
     * In the case that the desired chassis accelerations are zero (i.e. the robot
     * will be stationary),
     * the previously calculated module angle will be maintained.
     *
     * @param chassisAccleration     The desired chassis acceleration.
     * @param centerOfRotationMeters The center of rotation. For example, if you set
     *                               the center of
     *                               rotation at one corner of the robot and provide
     *                               a chassis acceleration that only has a dtheta
     *                               component, the robot will rotate around that
     *                               corner.
     * @return An array containing the module Acclerations. Use caution because
     *         these module accelerations are not
     *         normalized. Sometimes, a user input may cause one of the module
     *         accelerations to go above the
     *         attainable max acceleration. Use the
     *         {@link #desaturateWheelAccelerations(SwerveModuleAcceleration[], double)
     *         DesaturateWheelAccelerations} function to rectify this issue.
     */

    public SwerveModuleAcceleration[] toSwerveModuleAccelerations(
            ChassisAcceleration chassisAcceleration, Translation2d centerOfRotationMeters) {
        if (chassisAcceleration.a_xMetersPerSecondSquared == 0 && chassisAcceleration.a_yMetersPerSecondSquared == 0
                && chassisAcceleration.alphaRadiansPerSecondSquared == 0
                && chassisAcceleration.omegaRadiansPerSecond == 0) {
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
                chassisAcceleration.a_xMetersPerSecondSquared,
                chassisAcceleration.a_yMetersPerSecondSquared,
                chassisAcceleration.omegaRadiansPerSecond,
                chassisAcceleration.alphaRadiansPerSecondSquared);

        SimpleMatrix moduleAccelerationMatrix = m_forwardKinematics.mult(chassisAccelerationVector);

        m_moduleAccelerations = new SwerveModuleAcceleration[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            double x = moduleAccelerationMatrix.get(i * 2 + 0, 0);
            double y = moduleAccelerationMatrix.get(i * 2 + 1, 0);

            double speed = Math.hypot(x, y);
            Rotation2d angle = new Rotation2d(x, y);

            m_moduleAccelerations[i] = new SwerveModuleAcceleration(speed, angle);
        }

        return m_moduleAccelerations;
    }

    /**
     * Performs inverse kinematics. See
     * {@link #toSwerveModuleAccelerations(ChassisAcceleration, Translation2d)}
     * toSwerveModuleAccelerations for more information.
     *
     * @param chassisAcclerations The desired chassis speed.
     * @return An array containing the module states.
     */
    public SwerveModuleAcceleration[] toSwerveModuleAccelerations(ChassisAcceleration chassisAcceleration) {
        return toSwerveModuleAccelerations(chassisAcceleration, new Translation2d());
    }

    /**
     * Performs forward kinematics to return the resulting chassis acceleration from
     * the
     * given module accelerations.
     *
     * @param wheelAccelerations The state of the modules (as a
     *                           SwerveModuleAcceleration type) as
     *                           measured from
     *                           respective encoders and gyros. The order of the
     *                           swerve
     *                           module states should be same as
     *                           passed into the constructor of this class.
     * @return The resulting chassis acceleration.
     */
    public ChassisAcceleration toChassisAccelerations(SwerveModuleAcceleration... wheelAccelerations) {
        if (wheelAccelerations.length != m_numModules) {
            throw new IllegalArgumentException("Number of wheel accelerations must match number of modules");
        }
        SimpleMatrix moduleAccelerationsMatrix = new SimpleMatrix(m_numModules * 2, 1);

        for (int i = 0; i < m_numModules; i++) {
            moduleAccelerationsMatrix.set(i * 2 + 0, 0, wheelAccelerations[i].accelMetersPerSecondSquared
                    * wheelAccelerations[i].omegaRadiansPerSecond.getCos());
            moduleAccelerationsMatrix.set(i * 2 + 1, 0, wheelAccelerations[i].accelMetersPerSecondSquared
                    * wheelAccelerations[i].omegaRadiansPerSecond.getSin());
        }

        SimpleMatrix chassisAccelerationMatrix = m_forwardKinematics.mult(moduleAccelerationsMatrix);
        return new ChassisAcceleration(
                chassisAccelerationMatrix.get(0, 0),
                chassisAccelerationMatrix.get(1, 0),
                chassisAccelerationMatrix.get(2, 0),
                chassisAccelerationMatrix.get(3, 0));
    }

    /**
     * Renormalizes the wheel accelerations if any individual acceleration is above
     * the specified maximum.
     *
     * @param moduleAccelerations                   Reference to array of module
     *                                              accelerations. The array will be
     *                                              mutated with the
     *                                              normalized accelerations!
     * @param maxAccelerationMetersPerSecondSquared The absolute max acceleration
     *                                              that a module can reach.
     */

    public static void desaturateWheelAccelerations(
            SwerveModuleAcceleration[] moduleAccelerations, double maxAccelerationMetersPerSecondSquared) {
        double realMaxAcceleration = Collections.max(Arrays.asList(moduleAccelerations)).accelMetersPerSecondSquared;
        if (realMaxAcceleration > maxAccelerationMetersPerSecondSquared) {
            for (SwerveModuleAcceleration moduleAcceleration : moduleAccelerations) {
                moduleAcceleration.accelMetersPerSecondSquared = moduleAcceleration.accelMetersPerSecondSquared
                        / realMaxAcceleration * maxAccelerationMetersPerSecondSquared;
            }
        }
    }

    /**
     * This method is used to modify the module states with the given module
     * accelerations
     * 
     * @param moduleAccelerations The module accelerations to modify the module
     *                            states with
     * @param moduleStates        The module states to modify
     * @param dtSeconds           delta time in seconds, usually 0.02s or tick time
     * 
     * @return The modified module states
     */
    public SwerveModuleState[] modifyModuleStatesWithAccels(SwerveModuleAcceleration[] moduleAccelerations,
            SwerveModuleState[] moduleStates, double dtSeconds) {
        if (moduleAccelerations.length != moduleStates.length) {
            throw new IllegalArgumentException("Number of wheel accelerations must match number of wheel states");
        }
        if (moduleAccelerations.length != m_numModules) {
            throw new IllegalArgumentException("Number of wheel accelerations must match number of modules");
        }

        for (int i = 0; i < moduleAccelerations.length; i++) {
            moduleStates[i].speedMetersPerSecond += moduleAccelerations[i].accelMetersPerSecondSquared * dtSeconds;
            moduleStates[i].angle = moduleStates[i].angle
                    .rotateBy(moduleAccelerations[i].omegaRadiansPerSecond.times(dtSeconds));
        }

        return moduleStates;
    }
}
