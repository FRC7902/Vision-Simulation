package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import java.io.IOException;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Four-module swerve drivetrain built by YAGSL from the JSON files in src/main/deploy/swerve.
 *
 * <p>In simulation YAGSL creates a physics model (its bundled copy of maple-sim, running on the
 * dyn4j engine) that simulates the drive motors, steering motors, wheel encoders, gyro, wheel
 * grip and collisions with the 2026 field. That physics model is our <b>ground truth</b>.
 *
 * <p>This class only exposes <i>raw sensor readings</i>. It deliberately does not decide where the
 * robot is; that is {@link frc.robot.localization.LocalizationManager}'s job.
 */
public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;

  /**
   * Added to the raw gyro to get a field-relative heading for <i>driving</i>. Set on every reset.
   * Driving uses the clean gyro (not the drifted one) so the controls stay predictable when we
   * inject odometry error.
   */
  private Rotation2d driverHeadingOffset = Rotation2d.kZero;

  public SwerveSubsystem() {
    // INFO publishes module states for AdvantageScope's Swerve tab but not YAGSL's own Field2d,
    // which would otherwise be confused with our localization field.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.INFO;
    try {
      File directory = new File(Filesystem.getDeployDirectory(), Constants.Drive.CONFIG_DIRECTORY);
      swerveDrive =
          new SwerveParser(directory)
              .createSwerveDrive(Constants.Drive.MAX_SPEED_MPS, Constants.Drive.START_POSE);
    } catch (IOException e) {
      throw new RuntimeException("Could not read the YAGSL swerve configuration", e);
    }

    swerveDrive.setHeadingCorrection(false);
    // Cosine compensation fights the simulated module model; YAGSL recommends disabling it in sim.
    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);

    // By default YAGSL steps the physics on its own 250 Hz thread. We step it from periodic()
    // instead so physics, odometry and the camera stay in lock-step with the 20 ms robot loop.
    // stopOdometryThread() also switches maple-sim to 5 physics sub-steps per robot loop.
    swerveDrive.stopOdometryThread();
    driverHeadingOffset = Constants.Drive.START_POSE.getRotation().minus(getRawGyroYaw());
  }

  @Override
  public void periodic() {
    // Reads the sensors, updates YAGSL's internal estimator (unused here) and, in simulation,
    // advances the physics by one 20 ms robot loop.
    swerveDrive.updateOdometry();
  }

  /** Drives relative to the field, using the (undrifted) gyro to know which way is downfield. */
  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    swerveDrive.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getDriverHeading()));
  }

  public void stop() {
    swerveDrive.drive(new ChassisSpeeds());
  }

  /** Raw gyroscope yaw, counter-clockwise positive. */
  public Rotation2d getRawGyroYaw() {
    return swerveDrive.getYaw();
  }

  /** Heading used for field-relative driving. */
  public Rotation2d getDriverHeading() {
    return getRawGyroYaw().plus(driverHeadingOffset);
  }

  /** Raw wheel encoder distances and steering angles for each module (FL, FR, BL, BR). */
  public SwerveModulePosition[] getModulePositions() {
    return swerveDrive.getModulePositions();
  }

  /** Robot-relative velocity measured from the wheel encoders. */
  public ChassisSpeeds getMeasuredRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Where the robot <i>really</i> is according to the physics simulation. Only meaningful in
   * simulation; on a real robot nobody knows the true pose, so we fall back to YAGSL's estimate.
   */
  public Pose2d getGroundTruthPose() {
    return swerveDrive.getSimulationDriveTrainPose().orElseGet(swerveDrive::getPose);
  }

  /**
   * Teleports the simulated robot to {@code pose}, stops it, and sets the simulated gyro to match.
   * Callers must reset their estimators afterwards.
   */
  public void resetSimulationPose(Pose2d pose) {
    swerveDrive
        .getMapleSimDrive()
        .ifPresent(
            sim -> {
              sim.setRobotSpeeds(new ChassisSpeeds());
              sim.getGyroSimulation().setRotation(pose.getRotation());
            });
    swerveDrive.resetOdometry(pose); // also moves the maple-sim chassis
    swerveDrive.invalidateCache(); // make the next gyro read see the new heading
    driverHeadingOffset = pose.getRotation().minus(getRawGyroYaw());
  }
}
