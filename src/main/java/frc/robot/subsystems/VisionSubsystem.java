package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
  // Camera names - must match PhotonVision configuration
  private static final String FRONT_CAMERA_NAME = "frontCamera";
  private static final String BACK_CAMERA_NAME = "backCamera";

  // Front Camera mount position and rotation
  private static final double FRONT_CAM_X = 13.6; // inches forward
  private static final double FRONT_CAM_Y = -10.5; // inches
  private static final double FRONT_CAM_Z = 12.125; // inches up
  private static final double FRONT_CAM_ROLL = 0.0; // degrees
  private static final double FRONT_CAM_PITCH = 0.50; // degrees
  private static final double FRONT_CAM_YAW = 22.8; // degrees

  // Back Camera mount position and rotation
  private static final double BACK_CAM_X = 13.6; // inches forward
  private static final double BACK_CAM_Y = 10.5; // inches
  private static final double BACK_CAM_Z = 12.125; // inches up
  private static final double BACK_CAM_ROLL = -0.440; // degrees
  private static final double BACK_CAM_PITCH = -0.5; // degrees
  private static final double BACK_CAM_YAW = -22.8; // degrees

  // Standard Deviations Explanation:
  // These represent uncertainty in the vision measurements [X, Y, Theta]
  // Lower values = trust vision more, higher values = trust odometry more

  // TELEOP SINGLE TAG: More conservative since driver is controlling
  // X/Y: 0.1m = trust position within 10cm
  // Theta: 4.0 rad = low confidence in rotation (single tag is poor for rotation)
  private static final Matrix<N3, N1> SINGLE_TAG_TELEOP_STD_DEVS = VecBuilder.fill(0.1, 0.1, 4.0);

  // AUTO SINGLE TAG: Very conservative since auto needs reliability
  // X/Y: 4.0m = very low confidence in position
  // Theta: 8.0 rad = very low confidence in rotation
  private static final Matrix<N3, N1> SINGLE_TAG_AUTO_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);

  // MULTI TAG: High confidence because multiple tags = better triangulation
  // X/Y: 0.1m = trust position within 10cm
  // Theta: 0.25 rad = good confidence in rotation (~14 degrees)
  private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.25);

  // Reject single tags beyond this distance (they're too inaccurate)
  private static final double MAX_SINGLE_TAG_DISTANCE = 3.0; // meters

  // Transform from robot center to cameras
  private final Transform3d robotToFrontCamera =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(FRONT_CAM_X),
              Units.inchesToMeters(FRONT_CAM_Y),
              Units.inchesToMeters(FRONT_CAM_Z)),
          new Rotation3d(
              Units.degreesToRadians(FRONT_CAM_ROLL),
              Units.degreesToRadians(FRONT_CAM_PITCH),
              Units.degreesToRadians(FRONT_CAM_YAW)));

  private final Transform3d robotToBackCamera =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(BACK_CAM_X),
              Units.inchesToMeters(BACK_CAM_Y),
              Units.inchesToMeters(BACK_CAM_Z)),
          new Rotation3d(
              Units.degreesToRadians(BACK_CAM_ROLL),
              Units.degreesToRadians(BACK_CAM_PITCH),
              Units.degreesToRadians(BACK_CAM_YAW)));

  // PhotonVision components
  private final PhotonCamera frontCamera;
  private final PhotonCamera backCamera;
  private final PhotonPoseEstimator frontPoseEstimator;
  private final PhotonPoseEstimator backPoseEstimator;
  private final AprilTagFieldLayout fieldLayout;

  // Drive subsystem reference for pose updates
  private final Drive drive;

  // Cache for fast access - updated every loop
  private PhotonPipelineResult lastFrontResult = new PhotonPipelineResult();
  private PhotonPipelineResult lastBackResult = new PhotonPipelineResult();
  private Optional<EstimatedRobotPose> lastFrontEstimatedPose = Optional.empty();
  private Optional<EstimatedRobotPose> lastBackEstimatedPose = Optional.empty();
  private Matrix<N3, N1> lastFrontStdDevs = SINGLE_TAG_AUTO_STD_DEVS;
  private Matrix<N3, N1> lastBackStdDevs = SINGLE_TAG_AUTO_STD_DEVS;

  // Performance tracking
  private long lastFrontUpdateTimeNanos = 0;
  private long lastBackUpdateTimeNanos = 0;

  /**
   * Creates a new VisionSubsystem.
   *
   * @param drive The drive subsystem to send vision measurements to
   */
  public VisionSubsystem(Drive drive) {
    this.drive = drive;

    // Load the official 2026 Rebuilt Andymark field layout
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // Initialize cameras
    frontCamera = new PhotonCamera(FRONT_CAMERA_NAME);
    backCamera = new PhotonCamera(BACK_CAMERA_NAME);

    // Create pose estimators with field layout and camera transforms
    frontPoseEstimator = new PhotonPoseEstimator(fieldLayout, robotToFrontCamera);
    backPoseEstimator = new PhotonPoseEstimator(fieldLayout, robotToBackCamera);

    // Log camera configuration to SmartDashboard (once at startup)
    SmartDashboard.putString("Vision/FieldLayout", "2026 Rebuilt Andymark");
    SmartDashboard.putString("Vision/FrontCamera/Name", FRONT_CAMERA_NAME);
    SmartDashboard.putString(
        "Vision/FrontCamera/Position",
        String.format("X=%.1f\" Y=%.1f\" Z=%.1f\"", FRONT_CAM_X, FRONT_CAM_Y, FRONT_CAM_Z));
    SmartDashboard.putString(
        "Vision/FrontCamera/Rotation",
        String.format(
            "Roll=%.2f° Pitch=%.2f° Yaw=%.2f°", FRONT_CAM_ROLL, FRONT_CAM_PITCH, FRONT_CAM_YAW));

    SmartDashboard.putString("Vision/BackCamera/Name", BACK_CAMERA_NAME);
    SmartDashboard.putString(
        "Vision/BackCamera/Position",
        String.format("X=%.1f\" Y=%.1f\" Z=%.1f\"", BACK_CAM_X, BACK_CAM_Y, BACK_CAM_Z));
    SmartDashboard.putString(
        "Vision/BackCamera/Rotation",
        String.format(
            "Roll=%.2f° Pitch=%.2f° Yaw=%.2f°", BACK_CAM_ROLL, BACK_CAM_PITCH, BACK_CAM_YAW));
  }

  @Override
  public void periodic() {
    // Each camera update is independent and fast
    long startTime = System.nanoTime();
    updateCamera(frontCamera, frontPoseEstimator, "FrontCamera", true);
    lastFrontUpdateTimeNanos = System.nanoTime() - startTime;

    startTime = System.nanoTime();
    updateCamera(backCamera, backPoseEstimator, "BackCamera", false);
    lastBackUpdateTimeNanos = System.nanoTime() - startTime;

    // Log to SmartDashboard (once per periodic cycle)
    log();
  }

  /**
   * Process all unread camera results for accurate pose estimation. Uses multi-tag strategy with
   * fallback to single-tag lowest ambiguity.
   */
  private void updateCamera(
      PhotonCamera camera,
      PhotonPoseEstimator poseEstimator,
      String cameraName,
      boolean isFrontCamera) {

    try {
      // Get all unread results to process every frame
      var results = camera.getAllUnreadResults();

      for (var result : results) {
        // Cache latest result for getter methods
        if (isFrontCamera) {
          lastFrontResult = result;
        } else {
          lastBackResult = result;
        }

        // Check if we have targets
        if (!result.hasTargets()) {
          if (isFrontCamera) {
            lastFrontEstimatedPose = Optional.empty();
          } else {
            lastBackEstimatedPose = Optional.empty();
          }
          continue;
        }

        // Try multi-tag estimation first (most accurate)
        var estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);

        // If multi-tag fails, fall back to lowest ambiguity single-tag
        if (estimatedPose.isEmpty()) {
          estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
        }

        // Cache estimated pose
        if (isFrontCamera) {
          lastFrontEstimatedPose = estimatedPose;
        } else {
          lastBackEstimatedPose = estimatedPose;
        }

        // Process the pose if we got one
        estimatedPose.ifPresent(
            pose -> {
              // Calculate standard deviations based on tag count and distance
              var stdDevs = calculateStdDevs(pose, result);

              // Cache std devs
              if (isFrontCamera) {
                lastFrontStdDevs = stdDevs;
              } else {
                lastBackStdDevs = stdDevs;
              }

              // Send to drive if valid (not rejected)
              if (stdDevs.get(0, 0) < Double.MAX_VALUE) {
                drive.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(), pose.timestampSeconds, stdDevs);
              }
            });
      }
    } catch (Exception e) {
      DriverStation.reportError(
          "Error processing " + cameraName + " results: " + e.getMessage(), e.getStackTrace());
    }
  }

  /**
   * OPTIMIZED: Fast standard deviation calculation. Pre-computed values and minimal branching for
   * speed.
   */
  private Matrix<N3, N1> calculateStdDevs(
      EstimatedRobotPose estimatedPose, PhotonPipelineResult result) {

    int numTags = estimatedPose.targetsUsed.size();

    // SPEED: Early return for no tags
    if (numTags == 0) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    // SPEED: Pre-select std devs based on mode and tag count
    Matrix<N3, N1> baseStdDevs;
    if (numTags > 1) {
      baseStdDevs = MULTI_TAG_STD_DEVS; // Best accuracy
    } else {
      baseStdDevs =
          DriverStation.isAutonomous() ? SINGLE_TAG_AUTO_STD_DEVS : SINGLE_TAG_TELEOP_STD_DEVS;
    }

    // SPEED: Fast distance calculation
    double avgDistance = 0;
    int validTags = 0;
    for (var target : estimatedPose.targetsUsed) {
      var tagPose = fieldLayout.getTagPose(target.getFiducialId());
      if (tagPose.isPresent()) {
        avgDistance +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
        validTags++;
      }
    }

    if (validTags == 0) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    avgDistance /= validTags;

    // SPEED: Reject far single tags quickly
    if (numTags == 1 && avgDistance > MAX_SINGLE_TAG_DISTANCE) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    // SPEED: Simple scaling formula (pre-computed constant 1/30)
    return baseStdDevs.times(1.0 + (avgDistance * avgDistance * 0.03333));
  }

  /**
   * Log vision data to SmartDashboard (called once per periodic cycle to avoid duplicate logging)
   */
  public void log() {
    // Front Camera - Connection and basic status
    SmartDashboard.putBoolean("Vision Front Connected", frontCamera.isConnected());
    SmartDashboard.putBoolean("Vision Front Has Targets", lastFrontResult.hasTargets());
    SmartDashboard.putNumber("Vision Front Target Count", lastFrontResult.getTargets().size());

    // Front Camera - Target details (only if targets exist)
    if (lastFrontResult.hasTargets()) {
      var bestTarget = lastFrontResult.getBestTarget();
      SmartDashboard.putNumber("Vision Front Best Target ID", bestTarget.getFiducialId());
      SmartDashboard.putNumber("Vision Front Best Target Area", bestTarget.getArea());
      SmartDashboard.putNumber("Vision Front Best Target Yaw", bestTarget.getYaw());
    }

    // Front Camera - Estimated pose (only if pose exists)
    lastFrontEstimatedPose.ifPresent(
        pose -> {
          SmartDashboard.putString(
              "Vision Front Estimated Pose",
              String.format(
                  "X=%.2f Y=%.2f θ=%.1f°",
                  pose.estimatedPose.toPose2d().getX(),
                  pose.estimatedPose.toPose2d().getY(),
                  pose.estimatedPose.toPose2d().getRotation().getDegrees()));
          SmartDashboard.putNumber("Vision Front Tags Used", pose.targetsUsed.size());
          SmartDashboard.putBoolean(
              "Vision Front Valid", lastFrontStdDevs.get(0, 0) < Double.MAX_VALUE);
          SmartDashboard.putBoolean(
              "Vision Front Sent to Drive", lastFrontStdDevs.get(0, 0) < Double.MAX_VALUE);
        });

    // Back Camera - Connection and basic status
    SmartDashboard.putBoolean("Vision Back Connected", backCamera.isConnected());
    SmartDashboard.putBoolean("Vision Back Has Targets", lastBackResult.hasTargets());
    SmartDashboard.putNumber("Vision Back Target Count", lastBackResult.getTargets().size());

    // Back Camera - Target details (only if targets exist)
    if (lastBackResult.hasTargets()) {
      var bestTarget = lastBackResult.getBestTarget();
      SmartDashboard.putNumber("Vision Back Best Target ID", bestTarget.getFiducialId());
      SmartDashboard.putNumber("Vision Back Best Target Area", bestTarget.getArea());
      SmartDashboard.putNumber("Vision Back Best Target Yaw", bestTarget.getYaw());
    }

    // Back Camera - Estimated pose (only if pose exists)
    lastBackEstimatedPose.ifPresent(
        pose -> {
          SmartDashboard.putString(
              "Vision Back Estimated Pose",
              String.format(
                  "X=%.2f Y=%.2f θ=%.1f°",
                  pose.estimatedPose.toPose2d().getX(),
                  pose.estimatedPose.toPose2d().getY(),
                  pose.estimatedPose.toPose2d().getRotation().getDegrees()));
          SmartDashboard.putNumber("Vision Back Tags Used", pose.targetsUsed.size());
          SmartDashboard.putBoolean(
              "Vision Back Valid", lastBackStdDevs.get(0, 0) < Double.MAX_VALUE);
          SmartDashboard.putBoolean(
              "Vision Back Sent to Drive", lastBackStdDevs.get(0, 0) < Double.MAX_VALUE);
        });

    // Performance metrics
    SmartDashboard.putNumber("Vision Front Update Time (μs)", lastFrontUpdateTimeNanos / 1000.0);
    SmartDashboard.putNumber("Vision Back Update Time (μs)", lastBackUpdateTimeNanos / 1000.0);
  }

  // FAST GETTERS - No computation, just return cached values

  public boolean frontCameraHasTargets() {
    return lastFrontResult.hasTargets();
  }

  public boolean backCameraHasTargets() {
    return lastBackResult.hasTargets();
  }

  public int getFrontCameraBestTargetId() {
    return lastFrontResult.hasTargets() ? lastFrontResult.getBestTarget().getFiducialId() : -1;
  }

  public int getBackCameraBestTargetId() {
    return lastBackResult.hasTargets() ? lastBackResult.getBestTarget().getFiducialId() : -1;
  }

  public double getFrontCameraBestTargetArea() {
    return lastFrontResult.hasTargets() ? lastFrontResult.getBestTarget().getArea() : 0.0;
  }

  public double getBackCameraBestTargetArea() {
    return lastBackResult.hasTargets() ? lastBackResult.getBestTarget().getArea() : 0.0;
  }

  public PhotonPipelineResult getLastFrontResult() {
    return lastFrontResult;
  }

  public PhotonPipelineResult getLastBackResult() {
    return lastBackResult;
  }

  public Optional<EstimatedRobotPose> getLastFrontEstimatedPose() {
    return lastFrontEstimatedPose;
  }

  public Optional<EstimatedRobotPose> getLastBackEstimatedPose() {
    return lastBackEstimatedPose;
  }

  public Matrix<N3, N1> getLastFrontStdDevs() {
    return lastFrontStdDevs;
  }

  public Matrix<N3, N1> getLastBackStdDevs() {
    return lastBackStdDevs;
  }

  public AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }

  public Transform3d getRobotToFrontCamera() {
    return robotToFrontCamera;
  }

  public Transform3d getRobotToBackCamera() {
    return robotToBackCamera;
  }

  /** Get the last update time in microseconds for performance monitoring. */
  public double getFrontCameraUpdateTimeMicros() {
    return lastFrontUpdateTimeNanos / 1000.0;
  }

  public double getBackCameraUpdateTimeMicros() {
    return lastBackUpdateTimeNanos / 1000.0;
  }
}
