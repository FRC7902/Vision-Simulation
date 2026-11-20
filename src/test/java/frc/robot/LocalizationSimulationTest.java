package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import java.util.Arrays;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestMethodOrder;

/**
 * End-to-end test of the whole localization simulation, using the real Robot class, the real YAGSL physics
 * simulation, YALL's LimelightSim, and a simulated Xbox controller. The loop is paced in real time (20 ms per loop), exactly like
 * desktop simulation. (SimHooks.stepTiming was tried and blocked forever in this project, most
 * likely because a vendor library holds a native HAL notifier that never reports idle.)
 *
 * <p>The tests run in order and share one robot, like one continuous simulation session.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class LocalizationSimulationTest {
  private Robot robot;
  private RobotContainer container;
  private XboxControllerSim xbox;
  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();

  @BeforeAll
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.resetData();
    xbox = new XboxControllerSim(Constants.Controller.DRIVER_PORT);
    robot = new Robot();
    container = robot.getRobotContainer();
    enableTeleop();
  }

  @AfterAll
  void tearDown() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  // ------------------------------------------------------------------------------------------

  @Test
  @Order(1)
  void startsWithAllPosesAgreeingAndSeesTags() {
    step(0.5);
    Pose2d truth = pose2d("/Localization/GroundTruthPose");
    assertEquals(Constants.Drive.START_POSE.getX(), truth.getX(), 0.05);
    assertEquals(Constants.Drive.START_POSE.getY(), truth.getY(), 0.05);
    assertTrue(number("/Localization/OdometryErrorMeters") < 0.05);
    assertTrue(number("/Localization/FusedErrorMeters") < 0.10);

    assertTrue(bool("/Vision/HasTarget"), "camera should see the blue HUB tags from the start");
    long[] ids = ids();
    System.out.println("Visible tags at start: " + Arrays.toString(ids));
    assertTrue(ids.length >= 1);
    assertEquals(ids.length, nt.getEntry("/Vision/TagCount").getInteger(-1));
    assertTrue(nt.getEntry("/Vision/FusedMeasurements").getInteger(0) > 0);
    assertEquals("ACCEPTED_AND_FUSED", nt.getEntry("/Vision/Status").getString(""));

    // Raycasts: YALL's Field2d object and our 3D camera rays both contain the tags.
    assertEquals(2 * ids.length, field2dPoseCount("Limelight Raycasts"));
    assertEquals(2 * ids.length + 1, pose3dArray("/Vision/CameraRays").length);
  }

  @Test
  @Order(2)
  void cameraRaysEndExactlyOnTheRebuilt2026Tags() {
    step(0.2);
    Pose2d truth = pose2d("/Localization/GroundTruthPose");
    Pose3d[] rays = pose3dArray("/Vision/CameraRays");
    long[] ids = ids();
    assertTrue(ids.length > 0);

    // Ray start = the camera lens: robot pose + (0.25 m forward, 0.45 m up).
    Pose3d expectedCamera = new Pose3d(truth).plus(Constants.Vision.ROBOT_TO_CAMERA);
    AprilTagFieldLayout rebuilt = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    for (int i = 0; i < ids.length; i++) {
      Pose3d start = rays[2 * i];
      Pose3d end = rays[2 * i + 1];
      Pose3d tag = rebuilt.getTagPose((int) ids[i]).orElseThrow();
      System.out.printf(
          "Ray to tag %d: (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)%n",
          ids[i], start.getX(), start.getY(), start.getZ(), end.getX(), end.getY(), end.getZ());
      assertEquals(0.0, start.getTranslation().getDistance(expectedCamera.getTranslation()), 1e-6);
      assertEquals(0.45, start.getZ(), 1e-6);
      assertEquals(0.0, end.getTranslation().getDistance(tag.getTranslation()), 1e-6);
    }

    // Independent check against the published 2026 REBUILT (welded) tag map, e.g. tag 25 on the
    // blue HUB is at x=4.022, y=4.390, z=1.124 m (2026-rebuilt-welded.json).
    Pose3d tag25 = rebuilt.getTagPose(25).orElseThrow();
    assertEquals(4.022, tag25.getX(), 0.001);
    assertEquals(4.390, tag25.getY(), 0.001);
    assertEquals(1.124, tag25.getZ(), 0.001);

    // Every tag end point keeps its real height (not flattened to the floor).
    for (int i = 1; i < rays.length; i += 2) {
      assertTrue(rays[i].getZ() > 0.5, "ray must end at the tag center, above the floor");
    }
    // No 2D copy of the rays exists any more, so it can't be shown flat by mistake.
    assertFalse(nt.getTopic("/SmartDashboard/Localization Field/Camera Rays").exists());
  }

  @Test
  @Order(3)
  void visionTimestampMatchesRobotClock() {
    step(0.2);
    double now = Timer.getFPGATimestamp();
    double stamp = number("/Vision/MeasurementTimestamp");
    double latency = number("/Vision/LatencyMs") / 1000.0;
    System.out.printf("now=%.4f stamp=%.4f latency=%.4f%n", now, stamp, latency);
    // The newest frame was published by the previous loop's simulationPeriodic (one 20 ms loop
    // ago), and its timestamp is that publish time minus latency, on the SAME clock (FPGA time)
    // the pose estimator uses. A clock mismatch would show up here as a large offset.
    assertEquals(now - 0.02 - latency, stamp, 0.006);
  }

  @Test
  @Order(4)
  void xboxStickDrivesTheRobotAndOdometryFollows() {
    Pose2d before = pose2d("/Localization/GroundTruthPose");
    xbox.setLeftX(0.6); // left stick pushed to the RIGHT
    xbox.setLeftY(0.0);
    xbox.notifyNewData();
    step(1.0);
    xbox.setLeftX(0.0);
    xbox.notifyNewData();
    step(0.6);
    Pose2d after = pose2d("/Localization/GroundTruthPose");
    System.out.println("Strafe test: " + before + " -> " + after);
    // Pushing the stick to the right drives toward the field's -Y (driver's right).
    assertTrue(after.getY() < before.getY() - 0.5, "robot should strafe right");
    assertTrue(number("/Localization/OdometryErrorMeters") < 0.15);
  }

  @Test
  @Order(5)
  void rotatingAwayLosesTagsAndRaycasts() {
    container.resetPose();
    // Turn left (counter-clockwise) to face the side wall where no tags are visible.
    xbox.setRightX(-0.8);
    xbox.notifyNewData();
    boolean lost = false;
    for (int i = 0; i < 150 && !lost; i++) {
      step(0.02);
      lost = !bool("/Vision/HasTarget");
    }
    xbox.setRightX(0);
    xbox.notifyNewData();
    step(0.5);
    System.out.println(
        "Heading after turn: "
            + pose2d("/Localization/GroundTruthPose").getRotation().getDegrees()
            + " tags " + Arrays.toString(ids()));
    assertTrue(lost);
    assertFalse(bool("/Vision/HasTarget"));
    assertEquals(0, ids().length);
    assertEquals(0, field2dPoseCount("Limelight Raycasts"));
    assertEquals(0, pose3dArray("/Vision/CameraRays").length);
    assertEquals("NO_TARGET", nt.getEntry("/Vision/Status").getString(""));
    // Odometry keeps tracking without vision.
    assertTrue(number("/Localization/OdometryErrorMeters") < 0.15);
  }

  @Test
  @Order(6)
  void driftWithFusionDisabledCorruptsBothEstimates() {
    container.resetPose();
    Tunables.VISION_FUSION_ENABLED.set(false); // same as pressing B
    Tunables.DRIFT_ENABLED.set(true); // same as pressing X
    driveSquare();
    double odomError = number("/Localization/OdometryErrorMeters");
    double fusedError = number("/Localization/FusedErrorMeters");
    System.out.printf(
        "Drift, fusion off: odometry error %.2f m (%.1f deg), fused error %.2f m%n",
        odomError, number("/Localization/OdometryHeadingErrorDeg"), fusedError);
    assertTrue(odomError > 0.3, "odometry should visibly drift");
    assertTrue(fusedError > 0.3, "fused pose drifts too when fusion is off");
    assertTrue(nt.getEntry("/Vision/AcceptedMeasurements").getInteger(0) > 0);
    assertEquals(
        Math.abs(odomError - fusedError) < 0.05,
        true,
        "with fusion off both estimators see identical inputs");
  }

  @Test
  @Order(7)
  void reEnablingVisionCorrectsOnlyTheFusedPose() {
    // Drive back toward the HUB tags and face them.
    Tunables.DRIFT_ENABLED.set(false); // Y: no new error, old error stays
    double odomErrorBefore = number("/Localization/OdometryErrorMeters");
    faceTagsFromStart();
    assertTrue(bool("/Vision/HasTarget"));
    long fusedBefore = nt.getEntry("/Vision/FusedMeasurements").getInteger(0);
    Tunables.VISION_FUSION_ENABLED.set(true); // B
    step(1.5);
    double fusedError = number("/Localization/FusedErrorMeters");
    double odomError = number("/Localization/OdometryErrorMeters");
    System.out.printf(
        "After re-enabling vision: fused %.3f m / %.2f deg, odometry %.2f m (was %.2f)%n",
        fusedError, number("/Localization/FusedHeadingErrorDeg"), odomError, odomErrorBefore);
    assertTrue(nt.getEntry("/Vision/FusedMeasurements").getInteger(0) > fusedBefore);
    assertTrue(fusedError < 0.15, "vision should pull the fused pose back to the truth");
    assertTrue(number("/Localization/FusedHeadingErrorDeg") < 3.0);
    assertTrue(odomError > 0.3, "odometry-only pose must stay wrong");
  }

  @Test
  @Order(8)
  void megaTag2CorrectsTranslation() {
    container.resetPose();
    Tunables.USE_MEGATAG2.set(true);
    step(0.5);
    assertTrue(nt.getEntry("/Vision/UsingMegaTag2").getBoolean(false));
    assertEquals("ACCEPTED_AND_FUSED", nt.getEntry("/Vision/Status").getString(""));
    assertTrue(number("/Localization/FusedErrorMeters") < 0.15);
    Tunables.USE_MEGATAG2.set(false);
  }

  @Test
  @Order(9)
  void fullResetMakesEverythingAgree() {
    Tunables.DRIFT_ENABLED.set(true);
    Tunables.VISION_FUSION_ENABLED.set(false);
    Tunables.DRIFT_SEVERITY.set(2.5);
    driveSquare();
    assertTrue(number("/Localization/OdometryErrorMeters") > 0.3);

    Tunables.RESET_ALL.set(true); // same as pressing Start
    step(0.04);
    assertFalse(Tunables.RESET_ALL.get());
    assertTrue(Tunables.VISION_FUSION_ENABLED.get());
    assertFalse(Tunables.DRIFT_ENABLED.get());
    assertEquals(Constants.Drift.DEFAULT_SEVERITY, Tunables.DRIFT_SEVERITY.get());
    assertEquals(0.0, container.getDrift().getGyroErrorDegrees());
    assertTrue(number("/Localization/OdometryErrorMeters") < 0.02);
    assertTrue(number("/Localization/FusedErrorMeters") < 0.02);
    assertTrue(number("/Localization/OdometryHeadingErrorDeg") < 0.5);
    Pose2d truth = pose2d("/Localization/GroundTruthPose");
    assertEquals(Constants.Drive.START_POSE.getX(), truth.getX(), 0.02);
    assertEquals(Constants.Drive.START_POSE.getY(), truth.getY(), 0.02);
    step(0.5);
    assertTrue(number("/Localization/FusedErrorMeters") < 0.10);
  }

  @Test
  @Order(10)
  void wallCollisionErrorIsCorrectedOnlyInFusedPose() {
    container.resetPose();
    // Hit the blue HUB at an angle while spinning, then scrape along it.
    double[][] moves = {{-0.5, -0.9, 0.0}, {-0.6, -0.9, 0.8}, {0.9, -0.6, -0.8}, {-0.9, -0.6, 0.6}};
    for (double[] m : moves) {
      xbox.setLeftX(m[0]);
      xbox.setLeftY(m[1]);
      xbox.setRightX(m[2]);
      xbox.notifyNewData();
      step(1.0);
      System.out.printf(
          "  scrape step: truth %s odomErr %.3f%n",
          pose2d("/Localization/GroundTruthPose"), number("/Localization/OdometryErrorMeters"));
    }
    xbox.setLeftX(0);
    xbox.setRightX(0);
    xbox.notifyNewData();
    double odomAtWall = number("/Localization/OdometryErrorMeters");
    double fusedAtWall = number("/Localization/FusedErrorMeters");
    boolean tagsAtWall = bool("/Vision/HasTarget");
    // Back away so the HUB tags are in view again.
    xbox.setLeftY(0.6);
    xbox.notifyNewData();
    step(1.0);
    xbox.setLeftY(0);
    xbox.notifyNewData();
    step(1.5);
    double odom = number("/Localization/OdometryErrorMeters");
    double fused = number("/Localization/FusedErrorMeters");
    System.out.printf(
        "Wall: at wall odom %.3f fused %.3f tags %b | after backing off odom %.3f fused %.3f tags %s%n",
        odomAtWall, fusedAtWall, tagsAtWall, odom, fused, Arrays.toString(ids()));
    assertTrue(bool("/Vision/HasTarget"));
    assertTrue(odom > 0.2, "scraping the HUB should make the odometry-only pose wrong");
    assertTrue(fused < 0.15, "vision should correct the fused pose");
  }

  @Test
  @Order(11)
  void fusedPoseIsSteadyWhileStationaryButStillCorrectsQuickly() {
    container.resetPose();
    step(1.0);
    assertTrue(bool("/Vision/HasTarget"));
    double sx = 0, sxx = 0, sy = 0, syy = 0, st = 0, stt = 0, max = 0;
    int n = 150;
    for (int i = 0; i < n; i++) {
      step(0.02);
      Pose2d f = container.getLocalization().getFusedPose();
      Pose2d t = container.getSwerve().getGroundTruthPose();
      double dx = f.getX() - t.getX(), dy = f.getY() - t.getY();
      double dt = f.getRotation().minus(t.getRotation()).getDegrees();
      sx += dx; sxx += dx * dx; sy += dy; syy += dy * dy; st += dt; stt += dt * dt;
      max = Math.max(max, Math.hypot(dx, dy));
    }
    double stdX = Math.sqrt(sxx / n - sx * sx / n / n);
    double stdY = Math.sqrt(syy / n - sy * sy / n / n);
    double stdT = Math.sqrt(stt / n - st * st / n / n);

    // Knock the fused estimate 0.5 m / 10 deg off and time how long vision takes to fix it.
    Pose2d truth = container.getSwerve().getGroundTruthPose();
    container
        .getLocalization()
        .reset(
            container.getSwerve().getRawGyroYaw(),
            container.getSwerve().getModulePositions(),
            truth.plus(new Transform2d(0.4, 0.3, Rotation2d.fromDegrees(10))),
            Timer.getFPGATimestamp());
    double recover = Double.NaN;
    for (int i = 0; i < 100 && Double.isNaN(recover); i++) {
      step(0.02);
      Pose2d f = container.getLocalization().getFusedPose();
      Pose2d t = container.getSwerve().getGroundTruthPose();
      if (f.getTranslation().getDistance(t.getTranslation()) < 0.05
          && Math.abs(f.getRotation().minus(t.getRotation()).getDegrees()) < 1.0) {
        recover = (i + 1) * 0.02;
      }
    }
    System.out.printf(
        "Stationary jitter: x %.1f cm, y %.1f cm, heading %.2f deg, max %.1f cm; recovered in %.2f s%n",
        stdX * 100, stdY * 100, stdT, max * 100, recover);
    assertTrue(stdX < 0.015 && stdY < 0.015, "fused pose should not wiggle more than ~1.5 cm");
    assertTrue(stdT < 0.25, "fused heading should not wiggle more than ~0.25 deg");
    assertTrue(max < 0.05, "fused pose should never swing more than 5 cm while still");
    assertTrue(recover < 1.5, "vision must still correct a 0.5 m error within 1.5 s");
    container.resetPose();
  }

  // ------------------------------------------------------------------------------------------

  /** Drives a ~3 m x 1.5 m loop with the simulated controller. */
  private void driveSquare() {
    double[][] legs = {{0, -0.7}, {-0.7, 0}, {0, 0.7}, {0.7, 0}}; // (leftX, leftY)
    for (double[] leg : legs) {
      xbox.setLeftX(leg[0]);
      xbox.setLeftY(leg[1]);
      xbox.setRightX(0.25);
      xbox.notifyNewData();
      step(1.0);
    }
    xbox.setLeftX(0);
    xbox.setLeftY(0);
    xbox.setRightX(0);
    xbox.notifyNewData();
    step(0.6);
  }

  /** Teleports the robot back to the start pose WITHOUT resetting the estimators. */
  private void faceTagsFromStart() {
    container.getSwerve().resetSimulationPose(Constants.Drive.START_POSE);
    step(0.3);
  }

  private void enableTeleop() {
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    robot.teleopInit();
  }

  /** Runs the robot loop exactly like TimedRobot would, for {@code seconds}. */
  private void step(double seconds) {
    int loops = (int) Math.round(seconds / 0.02);
    for (int i = 0; i < loops; i++) {
      double loopStart = Timer.getFPGATimestamp();
      while (Timer.getFPGATimestamp() - loopStart < 0.02) {
        Thread.onSpinWait();
      }
      DriverStationSim.notifyNewData();
      robot.robotPeriodic();
      robot.simulationPeriodic();
      nt.flushLocal();
    }
  }

  private Pose2d pose2d(String topic) {
    return nt.getStructTopic(topic, Pose2d.struct).subscribe(new Pose2d()).get();
  }

  private Pose3d[] pose3dArray(String topic) {
    return nt.getStructArrayTopic(topic, Pose3d.struct).subscribe(new Pose3d[0]).get();
  }

  private double number(String topic) {
    return nt.getEntry(topic).getDouble(Double.NaN);
  }

  private boolean bool(String topic) {
    return nt.getEntry(topic).getBoolean(false);
  }

  private long[] ids() {
    return nt.getEntry("/Vision/VisibleTagIDs").getIntegerArray(new long[0]);
  }

  private int field2dPoseCount(String object) {
    double[] poses =
        nt.getEntry("/SmartDashboard/Localization Field/" + object).getDoubleArray(new double[0]);
    return poses.length / 3;
  }
}
