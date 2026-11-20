package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Every tunable number in the project lives here.
 *
 * <p>Units: meters, seconds, radians unless a name says otherwise. Field coordinates are WPILib
 * "blue origin": (0, 0) is the corner at the blue alliance wall, +X points toward the red alliance,
 * +Y points to the left when standing behind the blue driver stations.
 */
public final class Constants {
  private Constants() {}

  /** Swerve drivetrain. Geometry itself lives in src/main/deploy/swerve/*.json (read by YAGSL). */
  public static final class Drive {
    /** Folder under src/main/deploy that holds the YAGSL JSON configuration. */
    public static final String CONFIG_DIRECTORY = "swerve";

    /** Top translation speed we command. An MK4i L2 + NEO module tops out near 4.6 m/s. */
    public static final double MAX_SPEED_MPS = 4.0;

    /** Top rotation speed we command (270 deg/s feels controllable for new drivers). */
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.toRadians(270);

    /** Fraction of full speed used while the precision (left bumper) button is held. */
    public static final double PRECISION_SCALE = 0.35;

    /** Max change in joystick command per second (1.0 = full stick). Smooths acceleration. */
    public static final double TRANSLATION_SLEW_PER_SEC = 3.0;

    public static final double ROTATION_SLEW_PER_SEC = 4.0;

    /**
     * Where the robot starts (and returns to on reset): 2 m in front of the blue alliance wall,
     * centered on the field, facing the blue HUB. AprilTags 25 and 26 are visible from here.
     */
    public static final Pose2d START_POSE = new Pose2d(2.0, 4.03, Rotation2d.kZero);

    private Drive() {}
  }

  /** Driver controller. */
  public static final class Controller {
    /** Joystick port as shown in the simulator GUI "Joysticks" window. */
    public static final int DRIVER_PORT = 0;

    /** Stick values smaller than this are treated as zero (Xbox sticks rarely rest at exactly 0). */
    public static final double DEADBAND = 0.08;

    private Controller() {}
  }

  /** Limelight camera, simulated with YALL's LimelightSim. */
  public static final class Vision {
    /** NetworkTables name of the Limelight. A real Limelight's default name is "limelight". */
    public static final String LIMELIGHT_NAME = "limelight";

    /** Official 2026 REBUILT welded-field AprilTag layout (matches AdvantageScope's "2026 Field"). */
    public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2026RebuiltWelded;

    /**
     * Robot center (on the floor) to camera lens. 0.25 m forward, centered left/right, 0.45 m above
     * the floor, tilted 10 degrees upward. In WPILib a NEGATIVE pitch points the camera UP.
     */
    public static final Transform3d ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(0.25, 0.0, 0.45), new Rotation3d(0.0, Math.toRadians(-10.0), 0.0));

    /** Limelight 3/3G-like field of view. */
    public static final double HORIZONTAL_FOV_DEG = 63.3;

    public static final double VERTICAL_FOV_DEG = 49.7;

    public static final double RESOLUTION_WIDTH_PX = 1280;
    public static final double RESOLUTION_HEIGHT_PX = 960;

    /** Tags farther than this from the camera are not detected. */
    public static final double DEFAULT_MAX_DETECTION_RANGE_METERS = 5.0;

    /** Tags viewed at more than this angle from straight-on are not detected. */
    public static final double MAX_TAG_INCIDENCE_DEG = 75.0;

    /** Image processing time (mean and standard deviation), in milliseconds. */
    public static final double PIPELINE_LATENCY_MS = 25.0;

    public static final double PIPELINE_LATENCY_STDDEV_MS = 5.0;

    /** Time between the image being captured and processing starting, in milliseconds. */
    public static final double CAPTURE_LATENCY_MS = 10.0;

    /**
     * Noise added to each simulated pose estimate. YALL scales it by (1 + distance) / sqrt(tags),
     * so 0.01 gives about 2 cm per frame at 2 m with two tags, close to a real Limelight 3.
     * (YALL's own default, 0.03 m and 1.5 deg, made the fused pose visibly wiggle.)
     */
    public static final double DEFAULT_TRANSLATION_NOISE_METERS = 0.01;

    public static final double ROTATION_NOISE_DEG = 0.5;

    /** Noise on each tag's tx/ty angles. */
    public static final double ANGLE_NOISE_DEG = 0.15;

    /** Official FRC AprilTag size. */
    public static final double TAG_SIZE_METERS = Units.inchesToMeters(6.5);

    private Vision() {}
  }

  /** How vision measurements are filtered and how much they are trusted. */
  public static final class Fusion {
    /**
     * How much the pose estimator trusts wheel odometry (x m, y m, heading rad). Smaller = trust
     * odometry more. These are WPILib's documented starting values.
     */
    public static final double ODOMETRY_STDDEV_XY = 0.1;

    public static final double ODOMETRY_STDDEV_THETA = 0.1;

    /**
     * Vision standard deviation for one tag at 0 m. It grows with distance squared and shrinks
     * with more tags: stddev = base * (1 + DISTANCE_FACTOR * d^2) / tagCount.
     *
     * <p>The camera sends 50 measurements per second, so each one should only nudge the estimate a
     * little; the estimator then averages out the noise. These values were chosen by measurement
     * (see LocalizationSimulationTest): standing still the fused pose wobbles about 0.5 cm, and a
     * 0.5 m / 10 deg error is corrected in about 0.6 s. Larger = smoother but slower correction.
     */
    public static final double VISION_STDDEV_XY_BASE = 0.75;

    public static final double VISION_STDDEV_THETA_BASE = 1.5;

    public static final double VISION_DISTANCE_FACTOR = 0.1;

    /** MegaTag2 heading is just the robot's gyro echoed back, so it must never be trusted. */
    public static final double MEGATAG2_STDDEV_THETA = 1e6;

    // ---- Rejection rules. Deliberately lenient: they catch impossible data, not "surprising"
    // data. We do NOT reject measurements because they disagree with the current estimate,
    // because that would prevent vision from ever correcting large odometry drift.

    /** Reject poses farther than this outside the field boundary. */
    public static final double FIELD_MARGIN_METERS = 0.5;

    /** Reject poses that claim the robot is floating or underground by more than this. */
    public static final double MAX_POSE_Z_METERS = 0.75;

    /** Reject single-tag measurements with pose ambiguity above this (0 = certain, 1 = guess). */
    public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.7;

    /** Limelight recommends ignoring MegaTag2 while spinning faster than this. */
    public static final double MEGATAG2_MAX_YAW_RATE_DEG_PER_SEC = 720.0;

    private Fusion() {}
  }

  /** Deliberate odometry error, used to show how vision fusion corrects drift. */
  public static final class Drift {
    /** Default severity multiplier (0 = no effect, 1 = default, 3 = extreme). */
    public static final double DEFAULT_SEVERITY = 1.0;

    /**
     * Wheel encoders over-report distance by this fraction per unit of severity. This is what
     * happens with worn treads (smaller real wheel diameter) or wheel slip.
     */
    public static final double WHEEL_SCALE_ERROR_PER_SEVERITY = 0.15;

    /** The gyro heading drifts this many degrees per meter driven, per unit of severity. */
    public static final double GYRO_DRIFT_DEG_PER_METER_PER_SEVERITY = 4.0;

    private Drift() {}
  }
}
