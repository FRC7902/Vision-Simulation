# FRC Robot Localization Simulator

**Team 7902 Markham FireBirds: teaching odometry, AprilTags and sensor fusion**

This WPILib 2026 Java project simulates a swerve robot that you drive with an Xbox controller. A simulated Limelight camera on the robot detects AprilTags. In AdvantageScope you can watch three robots on the field:

| Robot | Color (included layout) | Meaning |
|---|---|---|
| **Ground truth** | solid robot | Where the robot *really* is, according to the physics simulation |
| **Odometry-only** | red ghost | Where the wheels and gyro *think* the robot is. Never uses the camera |
| **Vision-fused** | cyan ghost | Wheels and gyro **plus** AprilTag measurements |
| *Vision measurement* | yellow ghost | The camera's raw guess for this frame |

Green lines go from the robot to every AprilTag the camera currently sees. Magenta lines go from the camera lens to each tag.

No robot hardware is needed. Everything runs in WPILib desktop simulation.

---

## Contents

1. [What the simulator teaches](#1-what-the-simulator-teaches)
2. [Software requirements](#2-software-requirements)
3. [Dependency decisions](#3-dependency-decisions)
4. [Running the simulation](#4-running-the-simulation)
5. [AdvantageScope setup](#5-advantagescope-setup)
6. [Controller mapping](#6-controller-mapping)
7. [Teaching demonstration script (10–12 min)](#7-teaching-demonstration-script-1012-minutes)
8. [NetworkTables reference](#8-networktables-reference)
9. [Live tunables](#9-live-tunables)
10. [How the code is organized](#10-how-the-code-is-organized)
11. [Troubleshooting](#11-troubleshooting)
12. [Limitations](#12-limitations)
13. [Validation performed](#13-validation-performed)

---

## 1. What the simulator teaches

**Odometry** answers "how far did I move since last time?" On every 20 ms loop the robot reads how far each wheel rolled (drive encoders), which way each wheel points (steering encoders) and which way the robot faces (gyro). Adding up those small movements gives a position. It is fast, smooth and always available. But every small error is added in forever: a slipping wheel or a drifting gyro makes the estimate wander away from reality, and nothing in odometry can notice.

**AprilTag vision** answers "where am I on the field?" AprilTags are printed squares at known, published positions on the field. When the camera sees one, it can work out where the robot must be. This is an *absolute* measurement, so it does not accumulate error. However, it is noisy, it arrives late (latency), it gets worse with distance, and it disappears entirely when no tag is in view.

**Sensor fusion** combines the two. WPILib's `SwerveDrivePoseEstimator` follows odometry from loop to loop, and each time an accepted vision measurement arrives it nudges the estimate toward it. How hard it nudges depends on how much we trust each source (the *standard deviations*). The result is smooth like odometry and stays anchored to the field like vision.

The simulator lets students *break* odometry on purpose (X button), turn vision on and off (B button), and watch what happens.

---

## 2. Software requirements

| Item | Version used | Notes |
|---|---|---|
| WPILib | **2026.2.1** (GradleRIO `2026.2.1`) | Newest stable 2026 release at time of writing. **Not** 2027 |
| Java | **17** (the JDK bundled with WPILib 2026: Temurin 17.0.16) | The project targets Java 17 |
| Gradle | 8.11 (wrapper) | Comes with the project |
| Command framework | WPILib **Commands v2** (`WPILibNewCommands`) | Not Commands v3 |
| YALL | **2026.8.30** | Limelight library and Limelight simulation |
| YAGSL | **2026.8.04** (`swervelib:YAGSL-java`) | Swerve library, includes the physics simulation |
| YAMS | **not installed** | Not needed; see [Dependency decisions](#3-dependency-decisions) |
| dyn4j | 5.0.2 | Physics engine, listed by the YAGSL vendordep |
| REVLib | 2026.0.5 | Required by YAGSL (the config uses SPARK MAX + NEO) |
| ReduxLib | 2026.1.2 | Required by the YAGSL vendordep |
| ThriftyLib | 2026.1.2 | Required by the YAGSL vendordep |
| AdvantageScope | **26.0.0** (bundled with WPILib 2026.2.1) | Use the copy installed by the WPILib installer |

You do not need to download any of the libraries yourself. They are pinned in `vendordeps/` and Gradle downloads them on the first build.

---

## 3. Dependency decisions

| Library | Installed version | WPILib compatibility | Why it is needed | Source of compatibility information |
|---|---|---|---|---|
| WPILib / GradleRIO | 2026.2.1 | 2026 | Robot framework, simulation, NetworkTables, pose estimator | Gradle Plugin Portal: newest non-alpha/beta 2026 version is 2026.2.1 |
| YALL | 2026.8.30 | 2026 | Reads the Limelight, plus `LimelightSim` (simulated camera, raycasts on a Field2d) | `yall.json` says `"frcYear": "2026"`; jar is Java 17 bytecode linked against `edu.wpi.first.*` (2026 package names) |
| YAGSL | 2026.8.04 | 2026 | Swerve drive and desktop physics simulation (bundled maple-sim, `getSimulationDriveTrainPose()` for ground truth) | See explanation below |
| dyn4j | 5.0.2 | n/a (pure Java) | Physics engine used by YAGSL's bundled simulator | Listed in `yagsl-2026.8.04.json` |
| REVLib / ReduxLib / ThriftyLib | 2026.0.5 / 2026.1.2 / 2026.1.2 | 2026 | Listed as `requires` by the YAGSL 2026.8.04 vendordep | Each vendordep's `frcYear` is 2026 |
| YAMS | not installed | n/a | Not used by YAGSL 2026.8.04 | See below |

### How the YAGSL version was chosen

YAGSL has two release lines, and the newest releases target WPILib 2027:

- `https://cdn.yassrobotics.com/yagsl_commands2.json` is currently **2026.11.06**. It declares `"wpilibYear": "2027_alpha7"` and **requires YAMS**. Its release notes start with "Port 2026 to 2027_alpha7". **Rejected.**
- The classic `swervelib:YAGSL-java` line has `frcYear: 2026` up to 2026.10.03. Inspecting the published jars showed:
  - **2026.8.05** ("Update to 2027") declares `frcYear: "2026beta"`.
  - **2026.8.05 – 2026.10.03** are about 78 KB transitional builds. They contain only the JSON parser and a YAMS `NovaWrapper` class, and **no `SwerveDrive` class**, so they cannot drive a robot.
  - **2026.8.04** is the last full classic build (375 KB, the same class set as the 2026 in-season release 2026.4.1). It is Java 17 bytecode, uses only `edu.wpi.first.*` (2026) packages, and contains `SwerveDrive`, `SwerveParser` and the bundled maple-sim physics simulation.

So **YAGSL 2026.8.04 is the newest stable YAGSL release that actually works with WPILib 2026.**

> ⚠️ **Do not run "WPILib: Manage Vendor Libraries → Check for updates".**
> The online update URL inside the original YAGSL vendordep now serves the broken 2026.10.03 transitional build, and the YALL URL may move to 2027 in the future. To prevent accidental "upgrades", `vendordeps/yagsl-2026.8.04.json` has its `jsonUrl` set to `""`. Everything else in the file is unchanged from the official `yagsl-2026.8.04.json`.

### Why YAMS is not installed

1. YAGSL 2026.8.04's vendordep does **not** list YAMS in `requires`. It lists only ReduxLib, REVLib and ThriftyLib.
2. Its jar contains no `yams.*` classes and references no YAMS classes.
3. It has its own standalone simulation: a bundled copy of maple-sim under `swervelib.simulation.ironmaple`, using dyn4j for physics. Ground truth comes from `SwerveDrive.getSimulationDriveTrainPose()`.
4. Every published YAMS vendordep (`yams_commands2.json`, currently 2026.11.14) declares `wpilibYear: 2027_alpha7`.

So YAMS adds nothing this project needs and would pull in WPILib 2027 code.

### Why MapleSim is not installed separately

YAGSL 2026.8.04 already contains maple-sim (`swervelib.simulation.ironmaple`, including the 2026 REBUILT arena with field walls and HUB collisions). Adding the separate maple-sim vendordep would duplicate it.

---

## 4. Running the simulation

### 4.1 Open the project

1. Install **WPILib 2026** (2026.2.1 or newer 2026 patch) from the WPILib releases page, if it is not already installed.
2. Open **WPILib VS Code 2026** (not a normal VS Code, and not the 2027 one).
3. **File → Open Folder…** and pick this project folder.

### 4.2 Resolve dependencies and build

Open the command palette (Ctrl+Shift+P, or ⌘⇧P on macOS) and run **WPILib: Build Robot Code**. The first build downloads all libraries; it needs internet access once.

From a terminal you can use:

```bash
# macOS / Linux: use the JDK that WPILib installed
export JAVA_HOME=~/wpilib/2026/jdk
./gradlew build          # compiles AND runs the automated tests
```

```powershell
# Windows
$env:JAVA_HOME="C:\Users\Public\wpilib\2026\jdk"
.\gradlew.bat build
```

> The `JAVA_HOME` line matters if your computer has a newer Java (for example Java 24) installed. Gradle 8.11 and this project expect Java 17. WPILib VS Code sets this for you automatically.

### 4.3 Start desktop simulation

- In VS Code: command palette → **WPILib: Simulate Robot Code** → pick **Sim GUI** → **OK**.
- Or from a terminal: `./gradlew simulateJava`

The **Robot Simulation** window (Sim GUI) opens. The terminal should show `********** Robot program startup complete **********`. One `Loop time of 0.02s overrun` warning right at startup is normal: the first loop is slow while Java warms up.

### 4.4 Connect the Xbox controller

1. Plug the Xbox controller into the computer **before** starting the simulation (USB, or Bluetooth already paired).
2. In the Sim GUI, find the **System Joysticks** window. Your controller is listed there (for example "Xbox Wireless Controller").
3. Find the **Joysticks** window. **Drag** the controller from *System Joysticks* onto **Joystick[0]**. The program reads port 0 (`Constants.Controller.DRIVER_PORT`).
4. Tick **Map gamepad** under Joystick[0] (recommended). This makes the axes and buttons follow the standard Xbox layout on every operating system.
5. Move the sticks. The axis bars in the Joysticks window should move.

No controller? The program runs fine without one: the robot just stays still. You can also drag **Keyboard 0** onto Joystick[0] and drive with W/A/S/D (left stick). Turning with the keyboard needs axis 4 assigned under **DS → Keyboard 0 Settings**.

### 4.5 Enable the robot

In the **Robot State** window click **Teleoperated**. The robot only drives while enabled. The A/B/X/Y/Start/Back buttons also work while **Disabled**.

### 4.6 Drive

- Left stick: drive. Pushing **forward** drives away from the blue alliance wall (+X on the field), as if you were standing in the blue driver station.
- Right stick left/right: turn.
- Hold the **left bumper** for slow precision driving.

### 4.7 Confirm telemetry is updating

In the Sim GUI, open **NetworkTables → NetworkTables View** and expand `/Localization`. `GroundTruthPose`, `OdometryPose` and `FusedPose` should change as you drive. `/Vision/TagCount` should be **2** at the start position: the camera sees tags 25 and 26 on the blue HUB.

---

## 5. AdvantageScope setup

Use **AdvantageScope (WPILib)**, version 26.0.0, from the WPILib 2026 installer. In VS Code: command palette → **WPILib: Start Tool → AdvantageScope**.

### 5.1 Quick start: import the included layout

1. Start the simulation (section 4).
2. In AdvantageScope: **File → Connect to Simulator** (⌘⇧K / Ctrl+Shift+K).
3. **File → Import Layout…** and pick `advantagescope/LocalizationDemoLayout.json` from this project.
4. You get three tabs: **3D Localization**, **2D Localization** and **Localization Error**.

> The layout file was written by hand from AdvantageScope 26.0.0's own saved-state format and object-type definitions. It has **not** been visually checked by importing it into AdvantageScope. If anything is missing after import, add the objects by hand using 5.2 – 5.5 below (it takes about 3 minutes), then use **File → Export Layout…** to save a corrected copy.

### 5.2 3D Field tab (manual setup)

1. Click **+** (new tab) → **3D Field**.
2. At the bottom left, set the field to **2026 Field**. It matches the WPILib `k2026RebuiltWelded` AprilTag layout used by the code. Field size 651.22 × 317.68 in.
3. Drag these topics from the left sidebar into the **sources** list at the bottom of the tab, and pick the object type shown:

| Topic | Data type | Object type | Suggested style |
|---|---|---|---|
| `/Localization/GroundTruthPose` | Pose2d | **Robot** | model *2026 KitBot* |
| ↳ `/Vision/VisibleTagPoses` | Pose3d[] | **Vision Target** (drag it *onto* the robot so it becomes a child) | Green |
| `/Localization/OdometryPose` | Pose2d | **Ghost** | Red |
| `/Localization/FusedPose` | Pose2d | **Ghost** | Cyan |
| `/Localization/VisionPose` | Pose2d[] (0 or 1 pose) | **Ghost** | Yellow |
| `/Vision/CameraRays` | Pose3d[] | **Trajectory** | Magenta, Bold |
| `/Vision/CameraPose` | Pose3d | **Axes** | (shows exactly where the lens is) |

4. To change a color, click the colored icon next to the source (or right-click it).

**Two kinds of 3D raycast are available. Both were checked against AdvantageScope 26.0.0's renderer code:**

- **Vision Target** (child of the ground-truth robot): AdvantageScope natively draws one line from the robot (0.5 m above its origin) to each pose in `/Vision/VisibleTagPoses`. Lines appear and disappear as tags enter and leave view.
- **Trajectory** on `/Vision/CameraRays`: the code publishes the points `camera → tag₁ → camera → tag₂ → … → camera`. Trajectories are drawn as one connected line through all the points, so you see a separate ray from the **camera lens** to the center of each visible tag. When no tag is visible the array is empty and the rays vanish. These are true 3D rays: AdvantageScope's 3D renderer uses each point's x, y **and z** for 3D sources (only 2D sources are flattened to 0.02 m). Each ray therefore climbs from the lens (0.45 m) to the tag center, which is 0.55, 0.89 or 1.12 m high depending on the 2026 tag.

### 5.3 2D Field tab (manual setup)

1. **+** → **2D Field**, field **2026 Field**.
2. Add the same topics as above: Robot, Vision Target child, three Ghosts and the CameraRays Trajectory. The object types are the same.
3. Also add YALL's built-in raycast: expand `/SmartDashboard/Localization Field` and drag **`Limelight Raycasts`** in as a **Trajectory** (format *Pose2d*). `LimelightSim.withField2d(...)` draws this path, `robot → tag → robot → tag …`, so it shows rays from the robot center to each visible tag.

Instead of steps 2–3, you can drag the whole `/SmartDashboard/Localization Field` entry onto the 2D field. AdvantageScope then shows every Field2d object at once: `Robot` (ground truth), `Odometry Only (no vision)`, `Fused (odometry + vision)`, `Vision` and `Limelight Raycasts`. The Glass or Sim GUI Field2d widget shows the same data.

> ⚠️ **Use the `/SmartDashboard/Localization Field/…` entries only on the 2D Field.** A Field2d object is flat 2D data (`x, y, degrees, x, y, degrees, …`) with **no height**. On the 3D field:
> - with format *Pose2d*, everything is drawn flat on the floor (for example `Limelight Raycasts` point at the carpet under each tag);
> - with format *Pose3d*, AdvantageScope reads every 7 numbers as `x, y, z, qw, qx, qy, qz`, so the heading in degrees lands in the height slot and points fly into the sky.
>
> On the 3D field, always use the struct topics from 5.2 (`/Localization/...`, `/Vision/CameraRays`, `/Vision/VisibleTagPoses`). For this reason, the camera rays are published **only** as the 3D topic `/Vision/CameraRays`, not on the Field2d.

> ℹ️ **Only `Fused (odometry + vision)` is corrected by the camera.** `Odometry Only (no vision)` is deliberately never corrected, so after a collision or drift it *stays* wrong even while tags are visible. That is the comparison the demo is built around.

### 5.4 Localization Error tab (Line Graph)

1. **+** → **Line Graph**.
2. Left axis (meters): `/Localization/OdometryErrorMeters` and `/Localization/FusedErrorMeters`.
3. Right axis (degrees): `/Localization/OdometryHeadingErrorDeg` and `/Localization/FusedHeadingErrorDeg`.
4. Discrete fields: `/Vision/HasTarget`, `/Vision/FusionEnabled`, `/Simulation/DriftEnabled`, `/Vision/Status`.

This graph makes the lesson easy to see: during drift both error lines climb, and when vision is re-enabled the fused line drops back to almost zero while the odometry line stays high.

### 5.5 Other useful views

- **Swerve** tab: drag in `/SmartDashboard/swerve/advantagescope/currentStates` and `/SmartDashboard/swerve/advantagescope/desiredStates` (SwerveModuleState[] structs published by YAGSL) to see each wheel.
- Sidebar values worth showing students live: `/Vision/VisibleTagIDs`, `/Vision/TagCount`, `/Vision/Status`, `/Vision/LatencyMs`, `/Vision/AcceptedMeasurements`, `/Vision/FusedMeasurements`.

### 5.6 Save the layout

**File → Export Layout…** saves all tabs to a JSON file. AdvantageScope also remembers the current layout automatically next time it opens.

---

## 6. Controller mapping

| Input | Action |
|---|---|
| Left stick | Drive / strafe (field-relative, from the blue driver station's point of view) |
| Right stick X | Turn (rotate) |
| Left bumper (hold) | Precision mode (35% speed) |
| **A** | Reset robot pose: teleport back to the start position. Ground truth, odometry and fused pose all agree again, and accumulated drift is cleared. Demo settings are kept |
| **B** | Toggle vision fusion on/off (the camera keeps detecting either way) |
| **X** | Start injecting odometry drift |
| **Y** | Stop injecting drift. Error that already built up **stays**, because odometry cannot fix itself |
| **Start** | Reset the entire demo: pose, drift, counters, and all settings back to defaults |
| **Back** | Toggle MegaTag1 / MegaTag2 pose estimation (advanced) |

Driving details: 8% stick deadband; stick values are squared for finer control; speed and turn changes are rate-limited (slew rate) so the robot accelerates smoothly. Top speed is 4.0 m/s and 270°/s. All of these are in `Constants.java`.

---

## 7. Teaching demonstration script (10–12 minutes)

**Before class:** start the simulation, enable Teleoperated, open AdvantageScope with the layout, and put the **3D Localization** tab and the **Localization Error** graph side by side. Press **Start** once.

### Part 1: Odometry (≈2 min)

- **Do:** Drive a slow loop around the blue side of the field. Don't press any buttons.
- **Observe:** The solid robot, the red ghost (odometry) and the cyan ghost (fused) stay on top of each other. The error graph stays near zero.
- **Explain:** "The robot counts wheel rotations and reads the gyro 50 times per second, then adds up all those tiny movements. That's odometry."
- **Ask:** "If the robot only knows how much it moved, how does it know where it *started*?" (Answer: we tell it. The start pose is set in code.)

### Part 2: AprilTag visibility (≈2 min)

- **Do:** Press **A** to return to the start. Slowly turn left and right in place.
- **Observe:** Green and magenta lines connect the robot to tags 25 and 26 on the HUB. As you turn, lines disappear one at a time, then new ones appear when other tags come into the camera's 63° field of view. Watch `/Vision/VisibleTagIDs` change.
- **Explain:** "The camera only sees tags inside its cone of view and within 5 m. Each tag is at a known spot on the field, so seeing one tells us where *we* are."
- **Ask:** "Why do some tags disappear even though they're in front of us?" (Too far away, or seen too edge-on.)

### Part 3: Detecting ≠ using: turn fusion off (≈1.5 min)

- **Do:** Press **B**. `/Vision/FusionEnabled` turns false.
- **Observe:** The lines are still there. The camera still detects tags, and `/Vision/AcceptedMeasurements` keeps counting, but `/Vision/FusedMeasurements` stops increasing and `/Vision/Status` reads `ACCEPTED_FUSION_DISABLED`.
- **Explain:** "Seeing a tag and *using* that information are two separate steps. Right now the robot sees the tags but ignores them."

### Part 4: Break the odometry (≈2 min)

- **Do:** Press **X** (drift on). Drive a couple of big loops for about 15 seconds.
- **Observe:** The red and cyan ghosts both slide and rotate away from the real robot, and the error graph climbs. The real robot (ground truth) is unaffected.
- **Explain:** "We're pretending the wheels slip and the gyro drifts, like worn treads or bumping into another robot. Every small mistake gets added up forever."
- **Do:** Press **Y** (drift off) and keep driving.
- **Observe:** The ghosts stop drifting further, but they **do not come back**.
- **Ask:** "Why didn't they come back when we stopped the drift?" (Odometry has no way to notice its own mistakes.)

### Part 5: Vision to the rescue (≈2 min)

- **Do:** Drive so the camera faces a group of tags (the HUB, or the tags on the blue alliance wall). Then press **B** (fusion on).
- **Observe:** Within about a second the **cyan** (fused) ghost snaps back onto the real robot, and the fused error line drops to a few centimeters. The **red** (odometry-only) ghost stays wrong.
- **Explain:** "The pose estimator blends the two. Odometry gives smooth motion, and each AprilTag measurement nudges the estimate back toward the truth. Closer tags and more tags are trusted more."
- **Ask:** "Why don't we *only* use vision?" (It's noisy, it's late, and sometimes there are no tags in view. Next part!)

### Part 6: Losing vision (≈1.5 min)

- **Do:** Turn to face the side wall (no tags), then drive a short distance.
- **Observe:** All lines disappear, `/Vision/HasTarget` becomes false and `/Vision/Status` shows `NO_TARGET`. The fused robot keeps moving smoothly, using only odometry until tags come back into view.
- **Explain:** "Odometry and vision cover each other's weaknesses. Odometry works all the time but slowly drifts. Vision is only sometimes available but doesn't drift."

### Wrap-up (≈1 min)

- Press **Start** to reset everything.
- **Ask:** "On a real robot, what could make vision measurements *wrong*?" (Motion blur, a tag seen from very far away, a misconfigured camera position, or a reflection that looks like a tag.)
- **Bonus:** Raise `/Tuning/DriftSeverity` to 3, or raise `/Tuning/VisionStdDevScale` to 5 so vision is trusted less and corrects more slowly.

---

## 8. NetworkTables reference

All geometry is published as WPILib **struct** topics, which AdvantageScope reads natively.

| Topic | Type | Meaning |
|---|---|---|
| `/Localization/GroundTruthPose` | Pose2d | Physics-simulation pose (truth) |
| `/Localization/OdometryPose` | Pose2d | Wheel + gyro only, never corrected by vision |
| `/Localization/FusedPose` | Pose2d | `SwerveDrivePoseEstimator` output |
| `/Localization/VisionPose` | Pose2d[] | This frame's camera estimate (empty if none) |
| `/Localization/OdometryErrorMeters` | double | Distance odometry ↔ truth |
| `/Localization/FusedErrorMeters` | double | Distance fused ↔ truth |
| `/Localization/OdometryHeadingErrorDeg` | double | Heading error, degrees |
| `/Localization/FusedHeadingErrorDeg` | double | Heading error, degrees |
| `/Vision/HasTarget` | boolean | Stage 1: at least one AprilTag detected |
| `/Vision/TagCount` | int | Number of tags detected |
| `/Vision/VisibleTagIDs` | int[] | IDs of the detected tags |
| `/Vision/VisibleTagPoses` | Pose3d[] | Field poses of detected tags (for Vision Target lines) |
| `/Vision/CameraPose` | Pose3d | True field pose of the camera lens |
| `/Vision/CameraRays` | Pose3d[] | `camera → tag → camera → …` path (for Trajectory lines) |
| `/Vision/EstimateAvailable` | boolean | Stage 2: a new robot-pose estimate arrived this loop |
| `/Vision/MeasurementAccepted` | boolean | Stage 3: the estimate passed the sanity checks |
| `/Vision/MeasurementFused` | boolean | Stage 4: it was added to the pose estimator |
| `/Vision/Status` | string | `NO_TARGET`, `ACCEPTED_AND_FUSED`, `ACCEPTED_FUSION_DISABLED`, `REJECTED_…` |
| `/Vision/FusionEnabled` | boolean | Whether fusion is on |
| `/Vision/UsingMegaTag2` | boolean | MegaTag2 instead of MegaTag1 |
| `/Vision/AcceptedMeasurements` | int | Count since last full reset |
| `/Vision/RejectedMeasurements` | int | Count since last full reset |
| `/Vision/FusedMeasurements` | int | Count since last full reset |
| `/Vision/LatencyMs` | double | Capture + processing latency of the latest estimate |
| `/Vision/MeasurementTimestamp` | double | FPGA time (s) the latest image was captured |
| `/Vision/AverageTagDistanceMeters` | double | Average camera-to-tag distance |
| `/Vision/FusedStdDevs` | double[3] | x, y (m) and heading (rad) standard deviations used for the last fused measurement |
| `/Simulation/DriftEnabled` | boolean | Drift injection on |
| `/Simulation/DriftSeverity` | double | Drift multiplier |
| `/Simulation/InjectedGyroErrorDeg` | double | Total gyro error injected so far |
| `/SmartDashboard/Localization Field` | Field2d (2D only) | `Robot` (truth), `Odometry Only (no vision)`, `Fused (odometry + vision)`, `Vision`, `Limelight Raycasts` |
| `/limelight/*` | Limelight keys | Published by YALL `LimelightSim` exactly like a real Limelight (`tv`, `tid`, `rawfiducials`, `botpose_wpiblue`, `botpose_orb_wpiblue`, `json`, …) |

---

## 9. Live tunables

These are under `/Tuning`. Change them from the Sim GUI's NetworkTables view, from Glass, or from AdvantageScope's tuning mode. The controller buttons write to the same entries.

| Entry | Default | Effect |
|---|---|---|
| `VisionFusionEnabled` | true | Same as B |
| `UseMegaTag2` | false | Same as Back |
| `DriftEnabled` | false | Same as X / Y |
| `DriftSeverity` | 1.0 | 0 = no drift, 3 = extreme |
| `VisionStdDevScale` | 1.0 | Multiplies the x/y vision standard deviation. Larger = smoother but slower correction |
| `VisionHeadingStdDevScale` | 1.0 | Same for heading |
| `CameraMaxRangeMeters` | 5.0 | Detection range of the simulated camera |
| `CameraTranslationNoiseMeters` | 0.01 | Noise on simulated camera poses. Try 0.05 to show what a noisy camera does to the fused pose |
| `ResetPose` | false | Set true: same as A (clears itself) |
| `ResetDemo` | false | Set true: same as Start (clears itself) |

---

## 10. How the code is organized

```
src/main/java/frc/robot/
  Robot.java                 TimedRobot: calls the scheduler, then RobotContainer.periodic()
  RobotContainer.java        Creates everything, Xbox bindings, runs the localization pipeline
  Constants.java             ALL tunable numbers (speeds, camera mount, noise, drift, trust)
  Tunables.java              Live NetworkTables settings under /Tuning
  subsystems/
    SwerveSubsystem.java     YAGSL swerve: raw sensors, driving, ground truth, sim reset
    VisionSubsystem.java     YALL Limelight: detected tag IDs + new pose estimates
  localization/
    LocalizationManager.java Odometry-only + fused estimators, vision checks and fusion
    OdometryDriftSimulator.java  Corrupts wheel/gyro readings when drift is on
  simulation/
    SimulationManager.java   YALL LimelightSim, fed latency-delayed ground truth
    VisualizationPublisher.java  All NetworkTables output + the 2D Field2d
src/main/deploy/swerve/      YAGSL JSON configuration
advantagescope/LocalizationDemoLayout.json   AdvantageScope layout
src/test/java/frc/robot/     Automated tests
```

**One loop (20 ms):**

1. The command scheduler runs the drive command. `SwerveSubsystem.periodic()` calls YAGSL `updateOdometry()`, which in simulation also steps the physics 20 ms forward (5 sub-steps).
2. `RobotContainer.periodic()`:
   - It reads raw wheel positions and the gyro.
   - `OdometryDriftSimulator` corrupts the readings (only when drift is on).
   - **Both** estimators receive the same readings.
   - A new camera estimate, if there is one, goes through `LocalizationManager.processVisionEstimate()`: checks first, then fusion if enabled.
   - Telemetry is published.
3. `RobotContainer.simulationPeriodic()`: the simulated Limelight takes its next picture using the **ground-truth** pose.

### Simulated drivetrain

The robot is a generic 22.75 in × 22.75 in wheelbase (module centers ±11.375 in) swerve with MK4i-style L2 gearing: drive 6.75:1, steering 150/7:1, 4 in wheels. It uses SPARK MAX + NEO motors, Thrifty analog absolute encoders on MXP analog inputs 4–7, a WPILib analog gyro on analog 0, and weighs 115 lb. You can change all of this in `src/main/deploy/swerve/`. In simulation, YAGSL's bundled maple-sim models the motors, encoders, wheel grip (coefficient of friction 1.19), gyro and collisions with the 2026 field.

### Camera

The camera is mounted 0.25 m forward of robot center, centered left/right, 0.45 m above the floor, and tilted 10° up (`Constants.Vision.ROBOT_TO_CAMERA`). It has a 63.3° × 49.7° field of view (Limelight 3), 1280×960 resolution and a 5 m range, and ignores tags viewed more than 75° off-axis. Latency is 25 ± 5 ms processing plus 10 ms capture. Pose noise is 1 cm and 0.5°, scaled by YALL as (1 + distance) / √tags: about 2 cm per frame at 2 m with two tags. YALL's own default (3 cm, 1.5°) is noisier than a real Limelight 3 and made the fused pose visibly wiggle.

### Vision checks (deliberately lenient)

A measurement is rejected only if it:

- lies more than 0.5 m outside the field,
- claims the robot is more than 0.75 m above or below the floor,
- is a single tag with ambiguity above 0.7,
- has an average tag distance beyond the camera range,
- is MegaTag2 while spinning faster than 720°/s, or
- was captured before the last reset.

There is **no** "too far from the current estimate" rule. That rule is common on real robots, but here it would stop vision from ever correcting a large drift.

Trust: standard deviation = base × (1 + 0.1·d²) / tagCount, where d is the average tag distance in meters. The base is 0.75 m for x/y and 1.5 rad for heading. These look large, but the camera sends **50 measurements per second**: each one should only nudge the estimate a little, and the estimator averages out the noise. The values were chosen by measurement. Standing still, the fused pose wobbles about 0.5 cm (std) and never more than about 2 cm. A 0.5 m / 10° error is still corrected in about 0.6 s. MegaTag2 heading is never trusted (10⁶ rad). Odometry standard deviations are 0.1 / 0.1 / 0.1.

### Drift model

While drift is on:

- every wheel over-reports distance by **15% × severity** (worn tread or wheel slip),
- the gyro heading drifts by **4° per meter driven × severity**.

The errors are applied to the readings that go into *both* estimators. The physics simulation, and therefore ground truth and the camera, never sees them. Field-relative **driving** uses the clean gyro, so the controls stay predictable while the estimates drift.

---

## 11. Troubleshooting

| Problem | Fix |
|---|---|
| **Gradle build fails with "Unsupported class file major version" or a Gradle/Java error** | You are running Gradle with a newer Java. Use WPILib VS Code, or set `JAVA_HOME` to the WPILib 2026 JDK (section 4.2) |
| **Build can't download dependencies** | The first build needs internet. After that it works offline. Behind a school firewall, try a phone hotspot once |
| **Errors mentioning `org.wpilib…`, `2027`, YAMS, or Commands v3** | Someone updated a vendordep. Restore `vendordeps/` from git. Do **not** use "Check for updates" (section 3) |
| **`SwerveDrive` / `swervelib` classes not found** | `vendordeps/yagsl-2026.8.04.json` was replaced by a 2026.8.05+ build. Restore it |
| **YAGSL error "Conversion factors cannot be 0" or "Improper Module Location"** | A JSON file in `src/main/deploy/swerve/` was edited incorrectly. Check `physicalproperties.json` and the module `location` values |
| **YAGSL can't find `swervedrive.json`** | Run from the project root folder. The deploy folder must be `src/main/deploy/swerve` |
| **Xbox controller not detected** | Plug it in *before* starting the sim. Check the Sim GUI **System Joysticks** list. On macOS, try a USB cable; some Bluetooth pads aren't seen |
| **Controller detected but robot doesn't respond** | Drag it onto **Joystick[0]** in the Joysticks window. Make sure Teleoperated is enabled. Tick **Map gamepad** if the axes are mixed up |
| **Robot doesn't move at all** | The robot must be **enabled** (Teleoperated). Check that the drive command is listed under the Scheduler in the Sim GUI. If you're stuck against a wall or the HUB, press **A** |
| **Robot drives the wrong direction** | Field-relative controls assume the blue driver station: forward = toward the red alliance. Press **A** to reset heading |
| **No AprilTags detected** | Face tags within 5 m: from the start pose, face +X toward the HUB. Check `/Tuning/CameraMaxRangeMeters`. In the Sim GUI, check `/limelight/tv` |
| **Camera sees tags in odd places / wrong angles** | Check `Constants.Vision.ROBOT_TO_CAMERA`. Units are meters and radians, and **negative pitch tilts the camera up** |
| **3D rays end high in the sky, start in the wrong place, or lie flat on the floor** | You are showing a Field2d entry (anything under `/SmartDashboard/Localization Field/`) on the **3D** field. Field2d data has no height. Remove it and add **`/Vision/CameraRays`** (top-level `/Vision` table, type Pose3d[]) as a **Trajectory** (section 5.2). There is no format option to pick for it. The data is verified to start at the camera lens (0.45 m high) and end at the center of each 2026 REBUILT tag |
| **Ray start isn't exactly at the robot's center** | Expected for `/Vision/CameraRays`: they start at the **camera lens**, 0.25 m in front of the robot center, so the start point swings around as the robot turns. YALL's `Limelight Raycasts` start at the robot center but use the pose from about 35 ms ago (camera latency), so they trail slightly behind a fast-moving robot |
| **Robot and `Odometry Only` don't snap together when a tag is seen** | By design: the odometry-only pose never uses vision. Compare `Robot` with **`Fused (odometry + vision)`** (or `/Localization/FusedPose`). If the *fused* pose doesn't correct, check `/Vision/FusionEnabled` and `/Vision/Status`. Also note that while the robot is pressed against the HUB or a wall, the tags are usually too close or too steep to see; back away first |
| **No raycasts** | 2D: add `/SmartDashboard/Localization Field/Limelight Raycasts` or `/Vision/CameraRays` as a **Trajectory**. 3D: `/Vision/VisibleTagPoses` must be a **Vision Target** child *under* the robot, or add `/Vision/CameraRays` as a Trajectory. Lines appear only while `/Vision/HasTarget` is true |
| **Poses not updating** | Make sure the simulation is still running (terminal). In AdvantageScope use **File → Connect to Simulator**, not "Connect to Robot" |
| **AdvantageScope won't connect** | Start the simulation first, then connect. Use the WPILib-bundled AdvantageScope 26.x. Allow incoming connections if the firewall asks (NT4 port 5810 on 127.0.0.1) |
| **Drift doesn't seem to work** | Drift only grows while the robot **moves**. Check `/Simulation/DriftEnabled` and `/Tuning/DriftSeverity` > 0. Drive at least a few meters |
| **Vision doesn't correct the fused pose** | `/Vision/FusionEnabled` must be true and `/Vision/HasTarget` true. Check `/Vision/Status`: a `REJECTED_…` value says why. Large `/Tuning/VisionStdDevScale` makes correction slow |
| **Warning "Loop time of 0.02s overrun" once at startup** | Normal (Java warm-up). Repeated overruns mean the computer is overloaded |

---

## 12. Limitations

**What is simulated vs. real**

- Ground truth comes from YAGSL's bundled maple-sim physics. It is a good model, not a perfect one: no battery sag effects worth noting, no carpet differences, and simplified collisions.
- Wheel encoders and the gyro are simulated by maple-sim. Natural odometry error is small (some wheel slip under hard acceleration), which is why the demo injects deliberate drift.
- The motor, encoder and gyro *types* in the JSON (SPARK MAX/NEO, Thrifty analog, analog gyro) matter only on a real robot. The simulation uses maple-sim's motor models.

**Camera simplifications (from YALL `LimelightSim`)**

- The camera does not process images. YALL checks each tag against the camera's field of view, range and viewing angle, then publishes **ground truth plus Gaussian noise** as the pose estimate. It does not run solvePnP.
- There is no occlusion: a tag hidden behind the HUB or another robot is still "visible" if it is in the field of view and faces the camera.
- No motion blur, no exposure or lighting effects, and no false detections. Ambiguity is a simple random value, so the ambiguity rejection rule almost never fires in simulation.
- **MegaTag2** in YALL's sim takes the true translation (plus noise) and copies whatever heading the robot sends it. On a real robot a wrong heading also corrupts MegaTag2's translation. MegaTag1 is therefore the default, and it can correct heading drift too.
- **Latency:** LimelightSim reports a latency but builds its pose from the pose it is given. This project gives it the ground-truth pose from one *average* latency (35 ms) ago, so the timestamps are consistent to within the ±5 ms latency jitter.
- YALL's 2D raycasts start at the robot **center**, using that slightly delayed pose. `/Vision/CameraRays` starts at the actual camera lens.

**Drift simulation**

- Drift is a simple, deterministic model (wheel scale error plus gyro drift per meter). Real drift is messier: collisions, carpet seams, wheels lifting off.
- Field-relative driving uses the uncorrupted gyro, so drift does not change how the controls feel. On a real robot a drifting gyro would also tilt the driver's controls.

**Visualization**

- AdvantageScope's native 3D "Vision Target" lines start 0.5 m above the robot origin, not at the camera. Use the `/Vision/CameraRays` Trajectory for true camera-to-tag lines.
- The included AdvantageScope layout file has not been visually checked (see 5.1).

**Not verified**

- Driving with a **physical** Xbox controller was not tested during development. The same code path was tested with WPILib's `XboxControllerSim` (section 13).
- The project was not run on a real roboRIO. The code is written so `VisionSubsystem` would work with a real Limelight, but the swerve JSON describes a generic robot, not Team 7902's robot.

---

## 13. Validation performed

All of the following were run on macOS with WPILib 2026.2.1 and JDK 17.

- **Dependency resolution:** `./gradlew dependencies` resolves `wpilibj-java:2026.2.1`, `wpilibNewCommands-java:2026.2.1`, `YAGSL-java:2026.8.04`, `YALL-java:2026.8.30`, `dyn4j:5.0.2`, REVLib 2026.0.5, ReduxLib 2026.1.2 and ThriftyLib 2026.1.2. No 2027 or YAMS artifacts.
- **Compilation:** `./gradlew build` succeeds.
- **Desktop simulation:** `./gradlew simulateJava` starts the Sim GUI and serves NetworkTables on port 5810. The robot program starts with no errors (AdvantageScope connected automatically during the run).
- **Automated end-to-end test** (`src/test/java/frc/robot/LocalizationDemoTest.java`, 11 tests). It uses the real `Robot` class, YAGSL physics, YALL `LimelightSim`, a simulated Xbox controller and real-time 20 ms loops:
  1. At start, all three poses agree. Tags **25 and 26** are detected, the measurement is fused, and YALL's `Limelight Raycasts` and `/Vision/CameraRays` contain one ray per tag.
  2. The vision timestamp equals *(publish time − latency)* on the FPGA clock within 6 ms (measured: 1.3 ms), which proves the NT and estimator clocks agree.
  3. The simulated Xbox left stick strafes the robot about 1 m; odometry follows within 15 cm.
  4. Turning away (to about 118°) makes tags, raycasts and `HasTarget` disappear (`NO_TARGET`), and odometry keeps tracking.
  5. Drift with fusion off: odometry error **0.45 m / 22°** after one loop; the fused pose drifts identically.
  6. Re-enabling fusion while facing tags: fused error drops to **≈0.01 m / 0.06°** within 1.5 s, while odometry stays at **0.43 m** error.
  7. MegaTag2 measurements are accepted and fused.
  8. A full reset (`/Tuning/ResetDemo`, same as Start) restores all settings and makes all poses agree within 2 cm.
  9. **Ray geometry:** each `/Vision/CameraRays` ray starts exactly at the camera lens (robot pose + 0.25 m forward, 0.45 m up) and ends exactly on the 2026 REBUILT welded tag pose. For example, the ray to tag 25 runs (2.250, 4.030, 0.450) → (4.022, 4.390, 1.124).
  10. **Wall collision:** scraping along the blue HUB while spinning makes the odometry-only pose about 0.3 m wrong. After backing up into view of tags 25 and 26, the fused pose corrects to under 0.15 m within 1.5 s, while odometry-only stays wrong.
  11. **Stationary steadiness:** standing in view of tags 25 and 26 for 3 s, the fused pose wobbles 0.5–0.6 cm (std), 0.06° in heading, and at most 1.8 cm. A 0.5 m / 10° error is corrected in 0.56 s. (Before tuning: 3.7 cm std, up to 12.6 cm.)
- **Unit tests** (`OdometryDriftSimulatorTest`, 4 tests): drift scaling, gyro drift, "stop drift keeps old error", reset.
- **AdvantageScope object types** (Vision Target, Trajectory, Ghost, Axes) and their accepted data types were checked against the AdvantageScope 26.0.0 application code installed by WPILib.

Run the tests yourself with `./gradlew test`. The simulation test takes about 16 seconds, because it runs in real time.
