package frc.robot.simulation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import limelight.Limelight;
import limelight.sim.LimelightSim;
import limelight.sim.LimelightSimSettings;

/**
 * Runs the simulated camera. Only constructed in simulation.
 *
 * <p>YALL's {@link LimelightSim} is given the <b>ground-truth</b> robot pose from the physics
 * simulation, never an estimate. It decides which AprilTags are inside the camera's field of view
 * and range, and publishes a noisy robot pose on the same NetworkTables keys a real Limelight uses.
 *
 * <p><b>Latency.</b> A real camera's answer describes the robot as it was when the picture was
 * taken (about 35 ms ago). LimelightSim reports a latency but builds its pose from whatever pose
 * it is given. So we hand it the ground-truth pose from one average latency ago. The timestamp
 * YALL reports (now minus latency) then matches the moment the pose describes.
 */
public class SimulationManager {
  private static final double AVERAGE_LATENCY_SECONDS =
      (Constants.Vision.PIPELINE_LATENCY_MS + Constants.Vision.CAPTURE_LATENCY_MS) / 1000.0;

  private final LimelightSim limelightSim;
  private final TimeInterpolatableBuffer<Pose2d> groundTruthHistory =
      TimeInterpolatableBuffer.createBuffer(1.0);

  private double appliedRangeMeters = Double.NaN;
  private double appliedNoiseMeters = Double.NaN;

  public SimulationManager(Limelight limelight, AprilTagFieldLayout fieldLayout, Field2d field) {
    limelightSim =
        new LimelightSim(limelight)
            .withRobotToCameraTransform(Constants.Vision.ROBOT_TO_CAMERA)
            .withAprilTagFieldLayout(fieldLayout)
            // Draws "Limelight Raycasts" (robot -> tag -> robot -> tag ...) on this Field2d.
            .withField2d(field);
    applySettings(
        Constants.Vision.DEFAULT_MAX_DETECTION_RANGE_METERS,
        Constants.Vision.DEFAULT_TRANSLATION_NOISE_METERS);
  }

  /** Rebuilds the camera model if the range or noise tunables changed. */
  public void applySettings(double maxRangeMeters, double translationNoiseMeters) {
    if (maxRangeMeters == appliedRangeMeters && translationNoiseMeters == appliedNoiseMeters) {
      return;
    }
    appliedRangeMeters = maxRangeMeters;
    appliedNoiseMeters = translationNoiseMeters;
    limelightSim.withSettings(
        new LimelightSimSettings()
            .withResolution(
                Constants.Vision.RESOLUTION_WIDTH_PX, Constants.Vision.RESOLUTION_HEIGHT_PX)
            .withFOV(Constants.Vision.HORIZONTAL_FOV_DEG, Constants.Vision.VERTICAL_FOV_DEG)
            .withTagSize(Constants.Vision.TAG_SIZE_METERS)
            .withMaxDetectionRange(Math.max(0.1, maxRangeMeters))
            .withMaxTagIncidenceAngle(Constants.Vision.MAX_TAG_INCIDENCE_DEG)
            .withPipelineLatency(
                Constants.Vision.PIPELINE_LATENCY_MS, Constants.Vision.PIPELINE_LATENCY_STDDEV_MS)
            .withCaptureLatency(Constants.Vision.CAPTURE_LATENCY_MS)
            .withAngleNoise(Constants.Vision.ANGLE_NOISE_DEG)
            .withPoseNoise(Math.max(0.0, translationNoiseMeters), Constants.Vision.ROTATION_NOISE_DEG));
  }

  /**
   * Simulates one camera frame.
   *
   * @param groundTruthPose the physics simulation's pose right now
   * @param now current FPGA time in seconds
   */
  public void update(Pose2d groundTruthPose, double now) {
    groundTruthHistory.addSample(now, groundTruthPose);
    Pose2d poseWhenImageWasTaken =
        groundTruthHistory.getSample(now - AVERAGE_LATENCY_SECONDS).orElse(groundTruthPose);
    limelightSim.update(poseWhenImageWasTaken);
  }

  /** Forget pose history, e.g. after the robot is teleported by a reset. */
  public void clearHistory() {
    groundTruthHistory.clear();
  }
}
