package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.localization.LocalizationManager;
import frc.robot.localization.OdometryDriftSimulator;
import frc.robot.simulation.SimulationManager;
import frc.robot.simulation.VisualizationPublisher;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionEstimate;
import java.util.Optional;

/**
 * Creates every part of the robot, connects the Xbox controller, and runs the localization
 * pipeline once per 20 ms loop.
 *
 * <p>Order of one loop (see {@link Robot#robotPeriodic()}):
 *
 * <ol>
 *   <li>Command scheduler: drive command runs, swerve reads sensors and the physics steps forward.
 *   <li>{@link #periodic()}: drift is applied to the sensor readings, both estimators update, and
 *       any new camera estimate is checked and (maybe) fused. Telemetry is published.
 *   <li>{@link #simulationPeriodic()}: the simulated Limelight takes its next picture.
 * </ol>
 */
public class RobotContainer {
  private final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(Constants.Vision.FIELD_LAYOUT);

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();
  private final OdometryDriftSimulator drift = new OdometryDriftSimulator();
  private final LocalizationManager localization;
  private final VisualizationPublisher visualization = new VisualizationPublisher(fieldLayout);
  private final SimulationManager simulation; // null on a real robot

  private final CommandXboxController controller =
      new CommandXboxController(Constants.Controller.DRIVER_PORT);
  private final SlewRateLimiter xLimiter =
      new SlewRateLimiter(Constants.Drive.TRANSLATION_SLEW_PER_SEC);
  private final SlewRateLimiter yLimiter =
      new SlewRateLimiter(Constants.Drive.TRANSLATION_SLEW_PER_SEC);
  private final SlewRateLimiter rotLimiter =
      new SlewRateLimiter(Constants.Drive.ROTATION_SLEW_PER_SEC);

  private Optional<VisionEstimate> latestEstimate = Optional.empty();

  public RobotContainer() {
    Tunables.init();
    if (RobotBase.isSimulation()) {
      // With no controller plugged in, WPILib would print a warning every few seconds.
      DriverStation.silenceJoystickConnectionWarning(true);
      simulation =
          new SimulationManager(vision.getLimelight(), fieldLayout, visualization.getField());
    } else {
      simulation = null;
    }

    SwerveModulePosition[] positions = swerve.getModulePositions();
    drift.reset(positions);
    localization =
        new LocalizationManager(
            swerve.getKinematics(),
            fieldLayout,
            swerve.getRawGyroYaw(),
            positions,
            Constants.Drive.START_POSE);
    resetPose();

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(Commands.run(this::driveWithController, swerve).withName("Drive"));

    // Buttons also work while the robot is disabled.
    controller.a().onTrue(instant("Reset pose", this::resetPose));
    controller
        .b()
        .onTrue(
            instant(
                "Toggle vision fusion",
                () -> Tunables.VISION_FUSION_ENABLED.set(!Tunables.VISION_FUSION_ENABLED.get())));
    controller.x().onTrue(instant("Inject drift", () -> Tunables.DRIFT_ENABLED.set(true)));
    controller.y().onTrue(instant("Stop drift", () -> Tunables.DRIFT_ENABLED.set(false)));
    controller.start().onTrue(instant("Reset all", this::resetAll));
    controller
        .back()
        .onTrue(
            instant(
                "Toggle MegaTag2",
                () -> Tunables.USE_MEGATAG2.set(!Tunables.USE_MEGATAG2.get())));
  }

  private static Command instant(String name, Runnable action) {
    return Commands.runOnce(action).ignoringDisable(true).withName(name);
  }

  /** Left stick = drive (field-relative), right stick X = turn, left bumper = slow mode. */
  private void driveWithController() {
    double scale = controller.getHID().getLeftBumperButton() ? Constants.Drive.PRECISION_SCALE : 1.0;
    // Xbox sticks: pushing forward gives NEGATIVE Y, pushing left gives NEGATIVE X.
    // Field: +X is away from the blue wall, +Y is to the driver's left, CCW rotation is positive.
    double forward = xLimiter.calculate(shape(-controller.getLeftY()) * scale);
    double left = yLimiter.calculate(shape(-controller.getLeftX()) * scale);
    double turn = rotLimiter.calculate(shape(-controller.getRightX()) * scale);
    swerve.driveFieldRelative(
        new ChassisSpeeds(
            forward * Constants.Drive.MAX_SPEED_MPS,
            left * Constants.Drive.MAX_SPEED_MPS,
            turn * Constants.Drive.MAX_ANGULAR_SPEED_RAD_PER_SEC));
  }

  /** Deadband, then square (keeping the sign) for finer control near the center of the stick. */
  private static double shape(double value) {
    double v = MathUtil.applyDeadband(value, Constants.Controller.DEADBAND);
    return Math.copySign(v * v, v);
  }

  /** Runs every loop after the command scheduler. */
  public void periodic() {
    handleNetworkTablesButtons();

    // 1. Raw sensors -> (maybe) drifted sensors -> both estimators.
    OdometryDriftSimulator.Measurement measured =
        drift.apply(
            swerve.getModulePositions(),
            swerve.getRawGyroYaw(),
            Tunables.DRIFT_ENABLED.get(),
            Tunables.DRIFT_SEVERITY.get());
    localization.updateOdometry(measured.gyroYaw(), measured.modulePositions());

    // 2. Vision: detection -> estimate -> accepted? -> fused?
    boolean useMegaTag2 = Tunables.USE_MEGATAG2.get();
    double yawRate = swerve.getMeasuredRobotVelocity().omegaRadiansPerSecond;
    if (useMegaTag2) {
      vision.setRobotOrientation(localization.getFusedPose().getRotation(), yawRate);
    }
    latestEstimate = vision.pollNewEstimate(useMegaTag2);
    if (latestEstimate.isPresent()) {
      localization.processVisionEstimate(
          latestEstimate.get(),
          Tunables.VISION_FUSION_ENABLED.get(),
          Tunables.CAMERA_MAX_RANGE_METERS.get(),
          yawRate,
          Tunables.VISION_STDDEV_SCALE.get(),
          Tunables.VISION_HEADING_STDDEV_SCALE.get());
    } else {
      localization.noVisionThisLoop(vision.hasTarget());
    }

    // 3. Telemetry.
    visualization.publish(
        new VisualizationPublisher.Snapshot(
            swerve.getGroundTruthPose(),
            localization.getOdometryPose(),
            localization.getFusedPose(),
            latestEstimate,
            vision.getVisibleTagIds(),
            Tunables.VISION_FUSION_ENABLED.get(),
            useMegaTag2,
            localization.getLastStatus(),
            localization.getAcceptedCount(),
            localization.getRejectedCount(),
            localization.getFusedCount(),
            localization.getLastStdDevs(),
            Tunables.DRIFT_ENABLED.get(),
            Tunables.DRIFT_SEVERITY.get(),
            drift.getGyroErrorDegrees()));
  }

  /** Runs every loop in simulation only, after {@link #periodic()}. */
  public void simulationPeriodic() {
    simulation.applySettings(
        Tunables.CAMERA_MAX_RANGE_METERS.get(), Tunables.CAMERA_TRANSLATION_NOISE_METERS.get());
    // The camera ALWAYS sees the true robot pose, never an estimate.
    simulation.update(swerve.getGroundTruthPose(), Timer.getFPGATimestamp());
  }

  private void handleNetworkTablesButtons() {
    if (Tunables.RESET_ALL.get()) {
      Tunables.RESET_ALL.set(false);
      resetAll();
    }
    if (Tunables.RESET_POSE.get()) {
      Tunables.RESET_POSE.set(false);
      resetPose();
    }
  }

  /**
   * Puts the robot back at the start pose and makes ground truth, odometry and fused pose agree
   * again. Clears accumulated drift but keeps the settings (fusion on/off, drift on/off).
   */
  public void resetPose() {
    Pose2d pose = Constants.Drive.START_POSE;
    if (RobotBase.isSimulation()) {
      swerve.resetSimulationPose(pose);
      simulation.clearHistory();
    }
    SwerveModulePosition[] positions = swerve.getModulePositions();
    Rotation2d gyro = swerve.getRawGyroYaw();
    drift.reset(positions);
    localization.reset(gyro, positions, pose, Timer.getFPGATimestamp());
    xLimiter.reset(0);
    yLimiter.reset(0);
    rotLimiter.reset(0);
  }

  /** Full reset: pose, accumulated drift, counters and every setting. */
  public void resetAll() {
    Tunables.restoreDefaults();
    localization.resetCounters();
    resetPose();
  }

  public void stopDriving() {
    swerve.stop();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command: this project is a localization simulation.");
  }

  // ---- Accessors used by the automated tests.

  public SwerveSubsystem getSwerve() {
    return swerve;
  }

  public LocalizationManager getLocalization() {
    return localization;
  }

  public VisionSubsystem getVision() {
    return vision;
  }

  public OdometryDriftSimulator getDrift() {
    return drift;
  }
}
