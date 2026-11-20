package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.PoseEstimate;
import limelight.results.RawFiducial;

/**
 * Reads AprilTag detections and robot pose estimates from the Limelight using YALL.
 *
 * <p>This class only reads what the camera publishes on NetworkTables, so it works unchanged with a
 * real Limelight. In simulation, {@link frc.robot.simulation.SimulationManager} runs YALL's {@code
 * LimelightSim}, which publishes the same NetworkTables data a real camera would.
 *
 * <p>It answers two questions each loop, which are reported separately:
 *
 * <ol>
 *   <li>Which AprilTags does the camera see right now? ({@link #getVisibleTagIds()})
 *   <li>Did the camera produce a new robot pose estimate? ({@link #pollNewEstimate})
 * </ol>
 *
 * Whether that estimate is <i>trusted</i> and <i>used</i> is decided elsewhere, in {@link
 * frc.robot.localization.LocalizationManager}.
 */
public class VisionSubsystem extends SubsystemBase {
  /** One camera frame's robot pose estimate, copied out of YALL's reusable object. */
  public record VisionEstimate(
      Pose3d robotPose,
      double timestampSeconds,
      double latencyMs,
      int tagCount,
      double averageTagDistanceMeters,
      double maxAmbiguity,
      boolean isMegaTag2) {}

  private final Limelight limelight;
  private final LimelightPoseEstimator megaTag1Estimator;
  private final LimelightPoseEstimator megaTag2Estimator;
  private final DoubleArraySubscriber rawFiducialsSubscriber;

  private int[] visibleTagIds = new int[0];
  private double lastEstimateTimestamp = Double.NaN;

  public VisionSubsystem() {
    NetworkTable table =
        NetworkTableInstance.getDefault().getTable(Constants.Vision.LIMELIGHT_NAME);
    // YALL's Limelight constructor waits up to 2 s for a "getpipe" key and raises an error Alert
    // if it is missing. A real Limelight publishes it; in simulation we publish a placeholder
    // first (LimelightSim keeps it updated afterwards).
    if (!table.containsKey("getpipe")) {
      table.getEntry("getpipe").setDouble(0);
    }

    limelight = new Limelight(Constants.Vision.LIMELIGHT_NAME);
    megaTag1Estimator = limelight.createPoseEstimator(EstimationMode.MEGATAG1);
    megaTag2Estimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
    rawFiducialsSubscriber = table.getDoubleArrayTopic("rawfiducials").subscribe(new double[0]);
  }

  @Override
  public void periodic() {
    // rawfiducials = [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity] per visible tag.
    double[] raw = rawFiducialsSubscriber.get();
    int count = raw.length / 7;
    int[] ids = new int[count];
    for (int i = 0; i < count; i++) {
      ids[i] = (int) raw[i * 7];
    }
    visibleTagIds = ids;
  }

  public Limelight getLimelight() {
    return limelight;
  }

  /**
   * MegaTag2 needs the robot's heading from the gyro every loop. We send the robot's current best
   * heading estimate, exactly as a real robot would.
   */
  public void setRobotOrientation(Rotation2d heading, double yawRateRadPerSec) {
    limelight
        .getSettings()
        .withRobotOrientation(
            new Orientation3d(
                new Rotation3d(0, 0, heading.getRadians()),
                new AngularVelocity3d(
                    RadiansPerSecond.of(0),
                    RadiansPerSecond.of(0),
                    RadiansPerSecond.of(yawRateRadPerSec))));
  }

  /** IDs of AprilTags currently detected by the camera (empty when none). */
  public int[] getVisibleTagIds() {
    return visibleTagIds;
  }

  public boolean hasTarget() {
    return visibleTagIds.length > 0;
  }

  /**
   * Returns the camera's robot pose estimate if it is from a frame we have not seen before.
   * Returns empty when no tag is visible or the frame was already returned (prevents fusing the
   * same camera frame twice).
   */
  public Optional<VisionEstimate> pollNewEstimate(boolean useMegaTag2) {
    LimelightPoseEstimator estimator = useMegaTag2 ? megaTag2Estimator : megaTag1Estimator;
    Optional<PoseEstimate> maybeEstimate = estimator.getPoseEstimate();
    if (maybeEstimate.isEmpty()) {
      return Optional.empty();
    }
    // YALL reuses one PoseEstimate object per camera, so copy the values out immediately.
    PoseEstimate estimate = maybeEstimate.get();
    if (estimate.tagCount <= 0 || estimate.timestampSeconds == lastEstimateTimestamp) {
      return Optional.empty();
    }
    lastEstimateTimestamp = estimate.timestampSeconds;

    double maxAmbiguity = 0.0;
    for (RawFiducial fiducial : estimate.rawFiducials) {
      if (fiducial != null) {
        maxAmbiguity = Math.max(maxAmbiguity, fiducial.ambiguity);
      }
    }
    return Optional.of(
        new VisionEstimate(
            estimate.pose,
            estimate.timestampSeconds,
            estimate.latency,
            estimate.tagCount,
            estimate.avgTagDist,
            maxAmbiguity,
            estimate.isMegaTag2));
  }
}
