package frc.robot.util;

public class ChassisAccelerations {
    public double axMetersPerSecondSquared;
    public double ayMetersPerSecondSquared;
    public double alphaRadiansPerSecondSquared;
    public double omegaRadiansPerSecond;

    public ChassisAccelerations() {
    }

    /**
     * Constructs a ChassisSpeeds object.
     *
     * @param axMetersPerSecondSquared     Forward velocity.
     * @param ayMetersPerSecondSquared     Sideways velocity.
     * @param alphaRadiansPerSecondSquared Angular acceleration.
     * @param omegaRadiansPerSecond        Angular velocity.
     */
    public ChassisAccelerations(double axMetersPerSecondSquared, double ayMetersPerSecondSquared,
            double alphaRadiansPerSecondSquared,
            double omegaRadiansPerSecond) {
        this.axMetersPerSecondSquared = axMetersPerSecondSquared;
        this.ayMetersPerSecondSquared = ayMetersPerSecondSquared;
        this.alphaRadiansPerSecondSquared = alphaRadiansPerSecondSquared;
        this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }

    @Override
    public String toString() {
        return String.format(
                "ChassisSpeeds(Ax: %.2f m/s^2, Ay: %.2f m/s^2, Alpha: %.2f rad/s^2, Omega: %.2f rad/s)",
                axMetersPerSecondSquared, ayMetersPerSecondSquared, alphaRadiansPerSecondSquared,
                omegaRadiansPerSecond);
    }
}
