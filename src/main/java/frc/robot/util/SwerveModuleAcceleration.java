package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleAcceleration implements Comparable<SwerveModuleAcceleration> {

    public double accelMetersPerSecondSquared = 0;
    public Rotation2d velocityThetaRadiansPerSecond = new Rotation2d();

    /** Constructs a SwerveModuleState with zeros for speed and angle. */
    public SwerveModuleAcceleration() {}

    /**
     * Constructs a SwerveModuleAcceleration.
     *
     * @param accelMetersPerSecondSquared The acceleration of the module.
     * @param velocityThetaRadiansPerSecond The heading velocity of the module.
     */
    public SwerveModuleAcceleration(double accelMetersPerSecondSquared, Rotation2d velocityThetaRadiansPerSecond) {
        this.accelMetersPerSecondSquared = accelMetersPerSecondSquared;
        this.velocityThetaRadiansPerSecond = velocityThetaRadiansPerSecond;
    }

    @Override
    public int compareTo(SwerveModuleAcceleration other) {
        return Double.compare(this.accelMetersPerSecondSquared, other.accelMetersPerSecondSquared);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof SwerveModuleAcceleration) {
            SwerveModuleAcceleration other = (SwerveModuleAcceleration) obj;
            return Math.abs(other.accelMetersPerSecondSquared - accelMetersPerSecondSquared) < 1E-9
                    && velocityThetaRadiansPerSecond.equals(other.velocityThetaRadiansPerSecond);
        }
        return false;
    }

    @Override
    public String toString() {
      return String.format(
          "SwerveModuleState(Acceleration: %.2f m/s, Heading Velocity: %s)", accelMetersPerSecondSquared, velocityThetaRadiansPerSecond);
    }
}
