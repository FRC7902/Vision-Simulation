package frc.robot.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class OdometryDriftSimulatorTest {
  private static SwerveModulePosition[] wheels(double distance) {
    SwerveModulePosition[] p = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      p[i] = new SwerveModulePosition(distance, Rotation2d.kZero);
    }
    return p;
  }

  @Test
  void passesReadingsThroughWhenDisabled() {
    OdometryDriftSimulator drift = new OdometryDriftSimulator();
    drift.reset(wheels(0));
    var m = drift.apply(wheels(2.0), Rotation2d.fromDegrees(30), false, 1.0);
    assertEquals(2.0, m.modulePositions()[0].distanceMeters, 1e-9);
    assertEquals(30.0, m.gyroYaw().getDegrees(), 1e-9);
  }

  @Test
  void scalesWheelDistanceAndDriftsGyroWhenEnabled() {
    OdometryDriftSimulator drift = new OdometryDriftSimulator();
    drift.reset(wheels(0));
    var m = drift.apply(wheels(1.0), Rotation2d.kZero, true, 2.0);
    double expectedScale = 1 + 2.0 * Constants.Drift.WHEEL_SCALE_ERROR_PER_SEVERITY;
    assertEquals(expectedScale, m.modulePositions()[0].distanceMeters, 1e-9);
    assertEquals(
        2.0 * Constants.Drift.GYRO_DRIFT_DEG_PER_METER_PER_SEVERITY, m.gyroYaw().getDegrees(), 1e-9);
  }

  @Test
  void disablingKeepsAccumulatedErrorButAddsNoMore() {
    OdometryDriftSimulator drift = new OdometryDriftSimulator();
    drift.reset(wheels(0));
    drift.apply(wheels(1.0), Rotation2d.kZero, true, 1.0);
    double errorAfterDrift = drift.getGyroErrorDegrees();
    var m = drift.apply(wheels(3.0), Rotation2d.kZero, false, 1.0);
    assertEquals(errorAfterDrift, drift.getGyroErrorDegrees(), 1e-9);
    double expected = 1 + Constants.Drift.WHEEL_SCALE_ERROR_PER_SEVERITY + 2.0;
    assertEquals(expected, m.modulePositions()[0].distanceMeters, 1e-9);
  }

  @Test
  void resetClearsAllError() {
    OdometryDriftSimulator drift = new OdometryDriftSimulator();
    drift.reset(wheels(0));
    drift.apply(wheels(1.0), Rotation2d.kZero, true, 3.0);
    drift.reset(wheels(1.0));
    assertEquals(0.0, drift.getGyroErrorDegrees());
    var m = drift.apply(wheels(1.0), Rotation2d.kZero, false, 3.0);
    assertEquals(1.0, m.modulePositions()[0].distanceMeters, 1e-9);
  }
}
