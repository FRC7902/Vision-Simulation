package frc.robot;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Live-editable demo settings, published under the NetworkTables table {@code /Tuning}.
 *
 * <p>These entries are the single source of truth: controller buttons write to them, and you can
 * also change them from AdvantageScope (tuning mode), Glass, Elastic or the simulator GUI's
 * NetworkTables view. Every value is read again each robot loop.
 */
public final class Tunables {
  private static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable("Tuning");

  public static final BooleanEntry VISION_FUSION_ENABLED = bool("VisionFusionEnabled", true);
  public static final BooleanEntry USE_MEGATAG2 = bool("UseMegaTag2", false);
  public static final BooleanEntry DRIFT_ENABLED = bool("DriftEnabled", false);
  public static final DoubleEntry DRIFT_SEVERITY =
      number("DriftSeverity", Constants.Drift.DEFAULT_SEVERITY);
  public static final DoubleEntry VISION_STDDEV_SCALE = number("VisionStdDevScale", 1.0);
  public static final DoubleEntry VISION_HEADING_STDDEV_SCALE =
      number("VisionHeadingStdDevScale", 1.0);
  public static final DoubleEntry CAMERA_MAX_RANGE_METERS =
      number("CameraMaxRangeMeters", Constants.Vision.DEFAULT_MAX_DETECTION_RANGE_METERS);
  public static final DoubleEntry CAMERA_TRANSLATION_NOISE_METERS =
      number("CameraTranslationNoiseMeters", Constants.Vision.DEFAULT_TRANSLATION_NOISE_METERS);

  /** Set to true to reset only the robot pose (same as the A button). Clears itself. */
  public static final BooleanEntry RESET_POSE = bool("ResetPose", false);

  /** Set to true to reset the whole demo (same as the Start button). Clears itself. */
  public static final BooleanEntry RESET_DEMO = bool("ResetDemo", false);

  private Tunables() {}

  /** Restores every demo setting to its default value. */
  public static void restoreDefaults() {
    VISION_FUSION_ENABLED.set(true);
    USE_MEGATAG2.set(false);
    DRIFT_ENABLED.set(false);
    DRIFT_SEVERITY.set(Constants.Drift.DEFAULT_SEVERITY);
    VISION_STDDEV_SCALE.set(1.0);
    VISION_HEADING_STDDEV_SCALE.set(1.0);
    CAMERA_MAX_RANGE_METERS.set(Constants.Vision.DEFAULT_MAX_DETECTION_RANGE_METERS);
    CAMERA_TRANSLATION_NOISE_METERS.set(Constants.Vision.DEFAULT_TRANSLATION_NOISE_METERS);
  }

  /** Forces the class to load so every entry is published at startup. */
  public static void init() {}

  private static BooleanEntry bool(String name, boolean defaultValue) {
    BooleanEntry entry = TABLE.getBooleanTopic(name).getEntry(defaultValue);
    entry.set(defaultValue);
    return entry;
  }

  private static DoubleEntry number(String name, double defaultValue) {
    DoubleEntry entry = TABLE.getDoubleTopic(name).getEntry(defaultValue);
    entry.set(defaultValue);
    return entry;
  }
}
