package frc.robot.util;

public class SwerveModuleLateralAcceleration implements Comparable<SwerveModuleLateralAcceleration> {
  public double accelXMetersPerSecondSquared = 0;
  public double accelYMetersPerSecondSquared = 0;

  /**
   * Constructs a SwerveModuleLateralAcceleration.
   *
   * @param accelXMetersPerSecondSquared The x component of the lateral
   *                                     acceleration of the module relative to
   *                                     the field
   * @param accelYMetersPerSecondSquared The y component of the lateral
   *                                     acceleration of the module relative to
   *                                     the field
   */
  public SwerveModuleLateralAcceleration(double accelXMetersPerSecondSquared, double accelYMetersPerSecondSquared) {
    this.accelXMetersPerSecondSquared = accelXMetersPerSecondSquared;
    this.accelYMetersPerSecondSquared = accelYMetersPerSecondSquared;
  }
  /*Constructs a SwerveModuleLateralAcceleration with zeros for accelX and accelY */
  public SwerveModuleLateralAcceleration() {
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof SwerveModuleLateralAcceleration) {
      SwerveModuleLateralAcceleration other = (SwerveModuleLateralAcceleration) obj;
      return Math.abs(other.accelXMetersPerSecondSquared - accelXMetersPerSecondSquared) < 1E-9
          && Math.abs(other.accelYMetersPerSecondSquared - accelYMetersPerSecondSquared) < 1E-9;
    }
    return false;
  }

  @Override
  public int compareTo(SwerveModuleLateralAcceleration other) {

    double a = Math.sqrt(Math.pow(this.accelXMetersPerSecondSquared, 2) + Math.pow(this.accelYMetersPerSecondSquared, 2));
    double b = Math.sqrt(Math.pow(other.accelXMetersPerSecondSquared, 2) + Math.pow(other.accelYMetersPerSecondSquared, 2));

    return Double.compare(a,b);
  }

  @Override
  public String toString() {
    return String.format(
        "SwerveModuleState(accelX: %.2f m/s, accelY: %m/s)", accelXMetersPerSecondSquared, accelYMetersPerSecondSquared);
  }
}
