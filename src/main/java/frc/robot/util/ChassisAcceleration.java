package frc.robot.util;

public class ChassisAcceleration {
    public double a_xMetersPerSecondSquared;
    public double a_yMetersPerSecondSquared;
    public double alphaRadiansPerSecondSquared;
    public double omegaRadiansPerSecond;

    public ChassisAcceleration() {
    }

    /**
     * Constructs a ChassisSpeeds object.
     *
     * @param a_xMetersPerSecondSquared     Forward velocity.
     * @param a_yMetersPerSecondSquared     Sideways velocity.
     * @param alphaRadiansPerSecondSquared Angular acceleration.
     * @param omegaRadiansPerSecond        Angular velocity.
     */
    public ChassisAcceleration(double a_xMetersPerSecondSquared, double a_yMetersPerSecondSquared,
            double alphaRadiansPerSecondSquared,
            double omegaRadiansPerSecond) {
        this.a_xMetersPerSecondSquared = a_xMetersPerSecondSquared;
        this.a_yMetersPerSecondSquared = a_yMetersPerSecondSquared;
        this.alphaRadiansPerSecondSquared = alphaRadiansPerSecondSquared;
        this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }

    @Override
    public String toString() {
        return String.format(
                "ChassisSpeeds(A_x: %.2f m/s^2, A_y: %.2f m/s^2, Alpha: %.2f rad/s^2, Omega: %.2f rad/s)",
                a_xMetersPerSecondSquared, a_yMetersPerSecondSquared, alphaRadiansPerSecondSquared,
                omegaRadiansPerSecond);
    }
}
