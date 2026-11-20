package frc.robot.localization;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem.VisionEstimate;

/**
 * Keeps two completely independent guesses of where the robot is:
 *
 * <ul>
 *   <li><b>Odometry-only</b> ({@link SwerveDriveOdometry}): wheel encoders + gyro. Never sees the
 *       camera, so any error it accumulates stays forever.
 *   <li><b>Vision-fused</b> ({@link SwerveDrivePoseEstimator}): the same wheel/gyro data, plus
 *       AprilTag measurements blended in with a Kalman-filter-style weighting.
 * </ul>
 *
 * <p>Both receive the <i>same</i> (possibly drifted) sensor readings, so the only difference
 * between them is vision. Ground truth is never used here.
 *
 * <p>Each vision estimate passes through four stages, all reported to telemetry:
 *
 * <ol>
 *   <li>AprilTag detected (camera sees a tag)
 *   <li>Pose estimate available (camera computed a robot pose)
 *   <li>Measurement accepted (passed the sanity checks in {@link #check})
 *   <li>Measurement fused (accepted AND fusion is enabled, so it was added to the estimator)
 * </ol>
 */
public class LocalizationManager {
  /** Why a vision measurement was or was not used. */
  public enum VisionStatus {
    NO_TARGET,
    ACCEPTED_AND_FUSED,
    ACCEPTED_FUSION_DISABLED,
    REJECTED_OUTSIDE_FIELD,
    REJECTED_BAD_HEIGHT,
    REJECTED_HIGH_AMBIGUITY,
    REJECTED_TOO_FAR,
    REJECTED_SPINNING_TOO_FAST,
    REJECTED_BEFORE_RESET
  }

  private final SwerveDriveOdometry odometry;
  private final SwerveDrivePoseEstimator fusedEstimator;
  private final double fieldLength;
  private final double fieldWidth;

  private long acceptedCount = 0;
  private long rejectedCount = 0;
  private long fusedCount = 0;
  private VisionStatus lastStatus = VisionStatus.NO_TARGET;
  private double lastResetTimestamp = Double.NEGATIVE_INFINITY;
  private double[] lastStdDevs = new double[] {0, 0, 0};

  public LocalizationManager(
      SwerveDriveKinematics kinematics,
      AprilTagFieldLayout fieldLayout,
      Rotation2d gyroYaw,
      SwerveModulePosition[] modulePositions,
      Pose2d startPose) {
    odometry = new SwerveDriveOdometry(kinematics, gyroYaw, modulePositions, startPose);
    fusedEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            gyroYaw,
            modulePositions,
            startPose,
            VecBuilder.fill(
                Constants.Fusion.ODOMETRY_STDDEV_XY,
                Constants.Fusion.ODOMETRY_STDDEV_XY,
                Constants.Fusion.ODOMETRY_STDDEV_THETA),
            VecBuilder.fill(0.5, 0.5, 0.5)); // replaced per measurement below
    fieldLength = fieldLayout.getFieldLength();
    fieldWidth = fieldLayout.getFieldWidth();
  }

  /** Feeds one loop of (possibly drifted) wheel and gyro readings to BOTH estimators. */
  public void updateOdometry(Rotation2d gyroYaw, SwerveModulePosition[] modulePositions) {
    odometry.update(gyroYaw, modulePositions);
    fusedEstimator.update(gyroYaw, modulePositions);
  }

  /**
   * Runs one camera estimate through the acceptance checks and, if accepted and {@code
   * fusionEnabled}, blends it into the fused estimator. The odometry-only estimator is never
   * touched.
   */
  public VisionStatus processVisionEstimate(
      VisionEstimate estimate,
      boolean fusionEnabled,
      double maxRangeMeters,
      double yawRateRadPerSec,
      double stdDevScale,
      double headingStdDevScale) {
    VisionStatus status = check(estimate, maxRangeMeters, yawRateRadPerSec);
    if (status != null) {
      rejectedCount++;
      lastStatus = status;
      return status;
    }

    acceptedCount++;
    if (!fusionEnabled) {
      lastStatus = VisionStatus.ACCEPTED_FUSION_DISABLED;
      return lastStatus;
    }

    double[] stdDevs = computeStdDevs(estimate, stdDevScale, headingStdDevScale);
    lastStdDevs = stdDevs;
    fusedEstimator.addVisionMeasurement(
        estimate.robotPose().toPose2d(),
        estimate.timestampSeconds(),
        VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2]));
    fusedCount++;
    lastStatus = VisionStatus.ACCEPTED_AND_FUSED;
    return lastStatus;
  }

  /** Called on loops where the camera produced no new estimate. */
  public void noVisionThisLoop(boolean hasTarget) {
    if (!hasTarget) {
      lastStatus = VisionStatus.NO_TARGET;
    }
  }

  /** Returns null if the estimate is believable, otherwise the reason it was rejected. */
  private VisionStatus check(VisionEstimate estimate, double maxRangeMeters, double yawRate) {
    Pose3d pose = estimate.robotPose();
    double margin = Constants.Fusion.FIELD_MARGIN_METERS;
    if (estimate.timestampSeconds() < lastResetTimestamp) {
      // The image was taken before the robot was teleported by a reset.
      return VisionStatus.REJECTED_BEFORE_RESET;
    }
    if (pose.getX() < -margin
        || pose.getX() > fieldLength + margin
        || pose.getY() < -margin
        || pose.getY() > fieldWidth + margin) {
      return VisionStatus.REJECTED_OUTSIDE_FIELD;
    }
    if (Math.abs(pose.getZ()) > Constants.Fusion.MAX_POSE_Z_METERS) {
      return VisionStatus.REJECTED_BAD_HEIGHT;
    }
    if (estimate.tagCount() == 1
        && estimate.maxAmbiguity() > Constants.Fusion.MAX_SINGLE_TAG_AMBIGUITY) {
      return VisionStatus.REJECTED_HIGH_AMBIGUITY;
    }
    if (estimate.averageTagDistanceMeters() > maxRangeMeters) {
      return VisionStatus.REJECTED_TOO_FAR;
    }
    if (estimate.isMegaTag2()
        && Math.abs(Math.toDegrees(yawRate)) > Constants.Fusion.MEGATAG2_MAX_YAW_RATE_DEG_PER_SEC) {
      return VisionStatus.REJECTED_SPINNING_TOO_FAST;
    }
    return null;
  }

  /**
   * Vision gets less trustworthy the farther away the tags are, and more trustworthy with more
   * tags. Larger standard deviation = the estimator moves less toward the measurement.
   */
  private static double[] computeStdDevs(
      VisionEstimate estimate, double scale, double headingScale) {
    double distance = estimate.averageTagDistanceMeters();
    double factor =
        (1.0 + Constants.Fusion.VISION_DISTANCE_FACTOR * distance * distance)
            / Math.max(1, estimate.tagCount());
    double xy = Constants.Fusion.VISION_STDDEV_XY_BASE * factor * Math.max(0.01, scale);
    double theta =
        estimate.isMegaTag2()
            ? Constants.Fusion.MEGATAG2_STDDEV_THETA
            : Constants.Fusion.VISION_STDDEV_THETA_BASE * factor * Math.max(0.01, headingScale);
    return new double[] {xy, xy, theta};
  }

  /** Snaps both estimators to {@code pose}. */
  public void reset(
      Rotation2d gyroYaw, SwerveModulePosition[] modulePositions, Pose2d pose, double now) {
    odometry.resetPosition(gyroYaw, modulePositions, pose);
    fusedEstimator.resetPosition(gyroYaw, modulePositions, pose);
    lastResetTimestamp = now;
    lastStatus = VisionStatus.NO_TARGET;
  }

  public void resetCounters() {
    acceptedCount = 0;
    rejectedCount = 0;
    fusedCount = 0;
  }

  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  public Pose2d getFusedPose() {
    return fusedEstimator.getEstimatedPosition();
  }

  public long getAcceptedCount() {
    return acceptedCount;
  }

  public long getRejectedCount() {
    return rejectedCount;
  }

  public long getFusedCount() {
    return fusedCount;
  }

  public VisionStatus getLastStatus() {
    return lastStatus;
  }

  /** Standard deviations (x m, y m, heading rad) used for the most recently fused measurement. */
  public double[] getLastStdDevs() {
    return lastStdDevs;
  }
}
