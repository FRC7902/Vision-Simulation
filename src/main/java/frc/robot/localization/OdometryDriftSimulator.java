package frc.robot.localization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;

/**
 * Deliberately corrupts the wheel encoder and gyro readings <i>before</i> they reach the pose
 * estimators, so students can watch odometry drift away from reality.
 *
 * <p>Two realistic error sources are modeled while drift is enabled:
 *
 * <ul>
 *   <li><b>Wheel scale error</b>: each wheel over-reports how far it rolled (worn tread, wheel
 *       slip, wrong wheel diameter in the code).
 *   <li><b>Gyro drift</b>: the heading slowly rotates away from the truth as the robot drives.
 * </ul>
 *
 * <p>The physics simulation (ground truth) is never touched. Turning drift off stops <i>new</i>
 * error from being added, but error already accumulated stays, just like on a real robot. Only
 * vision or a pose reset can remove it.
 */
public class OdometryDriftSimulator {
  /** Sensor readings after corruption. These are what the estimators see. */
  public record Measurement(Rotation2d gyroYaw, SwerveModulePosition[] modulePositions) {}

  private double[] lastRawDistances = new double[0];
  private double[] reportedDistances = new double[0];
  private double gyroErrorRadians = 0.0;

  /** Forget all accumulated error and start passing readings through from these values. */
  public void reset(SwerveModulePosition[] rawPositions) {
    lastRawDistances = new double[rawPositions.length];
    reportedDistances = new double[rawPositions.length];
    for (int i = 0; i < rawPositions.length; i++) {
      lastRawDistances[i] = rawPositions[i].distanceMeters;
      reportedDistances[i] = rawPositions[i].distanceMeters;
    }
    gyroErrorRadians = 0.0;
  }

  /**
   * Turns this loop's raw sensor readings into the readings the estimators will see.
   *
   * @param enabled whether to add new error this loop
   * @param severity error multiplier (0 = none, 1 = normal demo)
   */
  public Measurement apply(
      SwerveModulePosition[] rawPositions, Rotation2d rawGyroYaw, boolean enabled, double severity) {
    if (rawPositions.length != lastRawDistances.length) {
      reset(rawPositions);
    }
    double clampedSeverity = Math.max(0.0, severity);
    double wheelScale =
        enabled ? 1.0 + Constants.Drift.WHEEL_SCALE_ERROR_PER_SEVERITY * clampedSeverity : 1.0;

    SwerveModulePosition[] reported = new SwerveModulePosition[rawPositions.length];
    double averageDistanceThisLoop = 0.0;
    for (int i = 0; i < rawPositions.length; i++) {
      double delta = rawPositions[i].distanceMeters - lastRawDistances[i];
      lastRawDistances[i] = rawPositions[i].distanceMeters;
      reportedDistances[i] += delta * wheelScale;
      averageDistanceThisLoop += Math.abs(delta) / rawPositions.length;
      // Steering angles are reported correctly; only distance is wrong.
      reported[i] = new SwerveModulePosition(reportedDistances[i], rawPositions[i].angle);
    }

    if (enabled) {
      gyroErrorRadians +=
          Math.toRadians(
              Constants.Drift.GYRO_DRIFT_DEG_PER_METER_PER_SEVERITY
                  * clampedSeverity
                  * averageDistanceThisLoop);
    }

    return new Measurement(rawGyroYaw.plus(Rotation2d.fromRadians(gyroErrorRadians)), reported);
  }

  /** Total gyro error injected so far, in degrees. */
  public double getGyroErrorDegrees() {
    return Math.toDegrees(gyroErrorRadians);
  }
}
