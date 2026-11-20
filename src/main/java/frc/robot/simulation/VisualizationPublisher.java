package frc.robot.simulation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.localization.LocalizationManager.VisionStatus;
import frc.robot.subsystems.VisionSubsystem.VisionEstimate;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Publishes everything AdvantageScope needs. Geometry uses WPILib struct topics, which
 * AdvantageScope reads natively as Pose2d / Pose3d / Pose3d[].
 *
 * <p>See README.md, "AdvantageScope setup", for which object type to use for each topic.
 */
public class VisualizationPublisher {
  /** Everything that is shown for one robot loop. */
  public record Snapshot(
      Pose2d groundTruth,
      Pose2d odometry,
      Pose2d fused,
      Optional<VisionEstimate> visionEstimate,
      int[] visibleTagIds,
      boolean fusionEnabled,
      boolean usingMegaTag2,
      VisionStatus visionStatus,
      long acceptedTotal,
      long rejectedTotal,
      long fusedTotal,
      double[] lastStdDevs,
      boolean driftEnabled,
      double driftSeverity,
      double injectedGyroErrorDeg) {}

  private final AprilTagFieldLayout fieldLayout;
  private final Field2d field = new Field2d();

  // ---- /Localization
  private final StructPublisher<Pose2d> groundTruthPose;
  private final StructPublisher<Pose2d> odometryPose;
  private final StructPublisher<Pose2d> fusedPose;
  private final StructArrayPublisher<Pose2d> visionPose;
  private final DoublePublisher odometryError;
  private final DoublePublisher fusedError;
  private final DoublePublisher odometryHeadingError;
  private final DoublePublisher fusedHeadingError;

  // ---- /Vision
  private final BooleanPublisher hasTarget;
  private final IntegerPublisher tagCount;
  private final IntegerArrayPublisher visibleTagIds;
  private final StructArrayPublisher<Pose3d> visibleTagPoses;
  private final StructPublisher<Pose3d> cameraPose;
  private final StructArrayPublisher<Pose3d> cameraRays;
  private final BooleanPublisher fusionEnabled;
  private final BooleanPublisher usingMegaTag2;
  private final BooleanPublisher estimateAvailable;
  private final StringPublisher status;
  private final BooleanPublisher measurementAccepted;
  private final BooleanPublisher measurementFused;
  private final IntegerPublisher acceptedCount;
  private final IntegerPublisher rejectedCount;
  private final IntegerPublisher fusedCount;
  private final DoublePublisher latencyMs;
  private final DoublePublisher measurementTimestamp;
  private final DoublePublisher averageTagDistance;
  private final DoubleArrayPublisher stdDevs;

  // ---- /Simulation
  private final BooleanPublisher driftEnabled;
  private final DoublePublisher driftSeverity;
  private final DoublePublisher injectedGyroError;

  public VisualizationPublisher(AprilTagFieldLayout fieldLayout) {
    this.fieldLayout = fieldLayout;
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    NetworkTable loc = nt.getTable("Localization");
    groundTruthPose = loc.getStructTopic("GroundTruthPose", Pose2d.struct).publish();
    odometryPose = loc.getStructTopic("OdometryPose", Pose2d.struct).publish();
    fusedPose = loc.getStructTopic("FusedPose", Pose2d.struct).publish();
    visionPose = loc.getStructArrayTopic("VisionPose", Pose2d.struct).publish();
    odometryError = loc.getDoubleTopic("OdometryErrorMeters").publish();
    fusedError = loc.getDoubleTopic("FusedErrorMeters").publish();
    odometryHeadingError = loc.getDoubleTopic("OdometryHeadingErrorDeg").publish();
    fusedHeadingError = loc.getDoubleTopic("FusedHeadingErrorDeg").publish();

    NetworkTable vision = nt.getTable("Vision");
    hasTarget = vision.getBooleanTopic("HasTarget").publish();
    tagCount = vision.getIntegerTopic("TagCount").publish();
    visibleTagIds = vision.getIntegerArrayTopic("VisibleTagIDs").publish();
    visibleTagPoses = vision.getStructArrayTopic("VisibleTagPoses", Pose3d.struct).publish();
    cameraPose = vision.getStructTopic("CameraPose", Pose3d.struct).publish();
    cameraRays = vision.getStructArrayTopic("CameraRays", Pose3d.struct).publish();
    fusionEnabled = vision.getBooleanTopic("FusionEnabled").publish();
    usingMegaTag2 = vision.getBooleanTopic("UsingMegaTag2").publish();
    estimateAvailable = vision.getBooleanTopic("EstimateAvailable").publish();
    status = vision.getStringTopic("Status").publish();
    measurementAccepted = vision.getBooleanTopic("MeasurementAccepted").publish();
    measurementFused = vision.getBooleanTopic("MeasurementFused").publish();
    acceptedCount = vision.getIntegerTopic("AcceptedMeasurements").publish();
    rejectedCount = vision.getIntegerTopic("RejectedMeasurements").publish();
    fusedCount = vision.getIntegerTopic("FusedMeasurements").publish();
    latencyMs = vision.getDoubleTopic("LatencyMs").publish();
    measurementTimestamp = vision.getDoubleTopic("MeasurementTimestamp").publish();
    averageTagDistance = vision.getDoubleTopic("AverageTagDistanceMeters").publish();
    stdDevs = vision.getDoubleArrayTopic("FusedStdDevs").publish();

    NetworkTable sim = nt.getTable("Simulation");
    driftEnabled = sim.getBooleanTopic("DriftEnabled").publish();
    driftSeverity = sim.getDoubleTopic("DriftSeverity").publish();
    injectedGyroError = sim.getDoubleTopic("InjectedGyroErrorDeg").publish();

    // 2D field: robot = ground truth; other objects are added below. In simulation YALL also
    // draws "Limelight Raycasts" on this same Field2d.
    // NOTE: Field2d objects are flat 2D data [x, y, degrees, ...]. For the 3D view use the
    // struct topics (/Localization/*, /Vision/CameraRays), which carry real 3D poses.
    SmartDashboard.putData("Localization Field", field);
  }

  /** The Field2d that YALL's LimelightSim draws its raycasts on. */
  public Field2d getField() {
    return field;
  }

  public void publish(Snapshot s) {
    // ---- Poses and errors
    groundTruthPose.set(s.groundTruth());
    odometryPose.set(s.odometry());
    fusedPose.set(s.fused());
    Pose2d[] visionPoses =
        s.visionEstimate().map(e -> new Pose2d[] {e.robotPose().toPose2d()}).orElse(new Pose2d[0]);
    visionPose.set(visionPoses);

    odometryError.set(distance(s.odometry(), s.groundTruth()));
    fusedError.set(distance(s.fused(), s.groundTruth()));
    odometryHeadingError.set(headingErrorDeg(s.odometry(), s.groundTruth()));
    fusedHeadingError.set(headingErrorDeg(s.fused(), s.groundTruth()));

    // ---- Camera and tags (camera pose uses ground truth: it is where the camera really is)
    Pose3d camera = new Pose3d(s.groundTruth()).plus(Constants.Vision.ROBOT_TO_CAMERA);
    List<Pose3d> tagPoses = new ArrayList<>();
    List<Pose3d> rays = new ArrayList<>();
    long[] ids = new long[s.visibleTagIds().length];
    for (int i = 0; i < s.visibleTagIds().length; i++) {
      ids[i] = s.visibleTagIds()[i];
      Optional<Pose3d> tag = fieldLayout.getTagPose(s.visibleTagIds()[i]);
      if (tag.isPresent()) {
        tagPoses.add(tag.get());
        // camera -> tag -> camera -> tag ... draws one line per tag as a single polyline.
        rays.add(camera);
        rays.add(tag.get());
      }
    }
    if (!rays.isEmpty()) {
      rays.add(camera);
    }

    hasTarget.set(ids.length > 0);
    tagCount.set(ids.length);
    visibleTagIds.set(ids);
    visibleTagPoses.set(tagPoses.toArray(new Pose3d[0]));
    cameraPose.set(camera);
    cameraRays.set(rays.toArray(new Pose3d[0]));

    // ---- Vision pipeline stages
    fusionEnabled.set(s.fusionEnabled());
    usingMegaTag2.set(s.usingMegaTag2());
    estimateAvailable.set(s.visionEstimate().isPresent());
    status.set(s.visionStatus().name());
    boolean fusedNow = s.visionStatus() == VisionStatus.ACCEPTED_AND_FUSED;
    boolean acceptedNow = fusedNow || s.visionStatus() == VisionStatus.ACCEPTED_FUSION_DISABLED;
    measurementAccepted.set(s.visionEstimate().isPresent() && acceptedNow);
    measurementFused.set(s.visionEstimate().isPresent() && fusedNow);
    acceptedCount.set(s.acceptedTotal());
    rejectedCount.set(s.rejectedTotal());
    fusedCount.set(s.fusedTotal());
    s.visionEstimate()
        .ifPresent(
            e -> {
              latencyMs.set(e.latencyMs());
              measurementTimestamp.set(e.timestampSeconds());
              averageTagDistance.set(e.averageTagDistanceMeters());
            });
    stdDevs.set(s.lastStdDevs());

    // ---- Drift
    driftEnabled.set(s.driftEnabled());
    driftSeverity.set(s.driftSeverity());
    injectedGyroError.set(s.injectedGyroErrorDeg());

    // ---- 2D field
    field.setRobotPose(s.groundTruth());
    // Names spell out the difference: odometry-only is NEVER corrected by vision.
    field.getObject("Odometry Only (no vision)").setPose(s.odometry());
    field.getObject("Fused (odometry + vision)").setPose(s.fused());
    field.getObject("Vision").setPoses(visionPoses);
    // Camera rays are deliberately NOT copied onto the Field2d: a Field2d has no height, so they
    // would be drawn on the floor in AdvantageScope's 3D view. Use /Vision/CameraRays instead.
  }

  private static double distance(Pose2d a, Pose2d b) {
    return a.getTranslation().getDistance(b.getTranslation());
  }

  private static double headingErrorDeg(Pose2d estimate, Pose2d truth) {
    return Math.abs(
        MathUtil.inputModulus(
            estimate.getRotation().minus(truth.getRotation()).getDegrees(), -180, 180));
  }
}
