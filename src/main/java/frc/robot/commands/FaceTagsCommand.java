//THIS IS AI GENERATED. WE REALLY NEED TO TEST THIS TO SEE HOW IT WORKDS WITH THE APRIL TAGS
//I SUGGEST PUTTING THIS ON 2 by 4 for testing and then set it on the ground
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlign;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * Command that auto-rotates robot to face hub AprilTags.
 * - Driver controls XY translation (field-oriented)
 * - Robot auto-rotates to face midpoint between tags
 * - If vision sees tags, auto-drives to target distance
 * - Uses real-time vision feedback to correct odometry drift
 */
public class FaceTagsCommand extends Command {
  private final Drive drive;
  private final VisionSubsystem vision;
  private final DoubleSupplier xSpeedSupplier;
  private final DoubleSupplier ySpeedSupplier;

  // Target AprilTag IDs (hub tags)
  private static final List<Integer> RED_HUB_TAGS = List.of(25, 26);
  private static final List<Integer> BLUE_HUB_TAGS = List.of(9, 10);

  // PID Controllers
  private final PIDController rotationController =
      new PIDController(AutoAlign.ROTATION_kP, AutoAlign.ROTATION_kI, AutoAlign.ROTATION_kD);
  private final PIDController distanceController =
      new PIDController(AutoAlign.DISTANCE_kP, AutoAlign.DISTANCE_kI, AutoAlign.DISTANCE_kD);

  // Target tracking
  private Translation2d targetMidpoint;
  private List<Integer> currentTagList;
  private boolean hasInitialized = false;

  /**
   * @param drive Drive subsystem
   * @param vision Vision subsystem
   * @param xSpeedSupplier Forward/backward speed from driver (-1 to 1)
   * @param ySpeedSupplier Left/right speed from driver (-1 to 1)
   */
  public FaceTagsCommand(
      Drive drive,
      VisionSubsystem vision,
      DoubleSupplier xSpeedSupplier,
      DoubleSupplier ySpeedSupplier) {
    this.drive = drive;
    this.vision = vision;
    this.xSpeedSupplier = xSpeedSupplier;
    this.ySpeedSupplier = ySpeedSupplier;

    addRequirements(drive);

    // Configure rotation controller
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Units.degreesToRadians(AutoAlign.ROTATION_TOLERANCE_DEG));

    // Configure distance controller
    distanceController.setTolerance(AutoAlign.DISTANCE_TOLERANCE_METERS);
  }

  @Override
  public void initialize() {
    hasInitialized = false;

    // Determine which tags to target based on alliance
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      SmartDashboard.putString("FaceTags/Status", "NO ALLIANCE");
      cancel();
      return;
    }

    // Select tag list based on alliance
    currentTagList =
        alliance.get() == Alliance.Red ? RED_HUB_TAGS : BLUE_HUB_TAGS;

    // Get tag positions from field layout and calculate midpoint
    List<Translation2d> tagPositions = new ArrayList<>();
    for (int tagId : currentTagList) {
      var tagPose = vision.getFieldLayout().getTagPose(tagId);
      if (tagPose.isPresent()) {
        tagPositions.add(tagPose.get().toPose2d().getTranslation());
      }
    }

    if (tagPositions.isEmpty()) {
      SmartDashboard.putString("FaceTags/Status", "TAGS NOT IN LAYOUT");
      cancel();
      return;
    }

    // Calculate midpoint between all visible tags
    targetMidpoint = calculateMidpoint(tagPositions);

    // Reset controllers
    rotationController.reset();
    distanceController.reset();

    hasInitialized = true;
    SmartDashboard.putString("FaceTags/Status", "ACTIVE");
    SmartDashboard.putString("FaceTags/TargetTags", currentTagList.toString());
    SmartDashboard.putString(
        "FaceTags/TargetMidpoint",
        String.format("X=%.2f Y=%.2f", targetMidpoint.getX(), targetMidpoint.getY()));
  }

  @Override
  public void execute() {
    if (!hasInitialized) return;

    // Get current robot pose from odometry (with vision corrections)
    Pose2d currentPose = drive.getPose();
    Translation2d robotPosition = currentPose.getTranslation();

    // Calculate desired heading to face midpoint
    Translation2d robotToTarget = targetMidpoint.minus(robotPosition);
    double currentDistance = robotToTarget.getNorm();
    Rotation2d desiredHeading = robotToTarget.getAngle();

    // Check if we can see any target tags
    List<Integer> visibleTags = getVisibleTags();
    boolean seesAnyTag = !visibleTags.isEmpty();

    // Update target midpoint from vision if we see tags (real-time correction)
    if (seesAnyTag) {
      Translation2d visionCorrectedMidpoint = calculateVisionCorrectedMidpoint(visibleTags);
      if (visionCorrectedMidpoint != null) {
        targetMidpoint = visionCorrectedMidpoint;
        robotToTarget = targetMidpoint.minus(robotPosition);
        currentDistance = robotToTarget.getNorm();
        desiredHeading = robotToTarget.getAngle();
      }
    }

    // Calculate rotation speed to face target
    double rotationSpeed =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), desiredHeading.getRadians());

    // Get driver translation inputs (field-oriented)
    double xSpeed = xSpeedSupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec();
    double ySpeed = ySpeedSupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec();

    // Auto-drive to target distance if:
    // 1. We see at least one tag
    // 2. We're within reasonable range
    // 3. Driver isn't providing forward/back input
    boolean shouldAutoDrive =
        seesAnyTag
            && currentDistance < AutoAlign.VISION_DISTANCE_THRESHOLD_METERS
            && Math.abs(xSpeed) < 0.1;

    if (shouldAutoDrive) {
      // Override X speed to maintain target distance
      double distanceError = currentDistance - AutoAlign.HUB_DISTANCE_METERS;
      double autoXSpeed = distanceController.calculate(0, -distanceError);

      // Limit auto speed
      autoXSpeed = Math.max(-2.0, Math.min(2.0, autoXSpeed));

      // Blend with driver input (driver can still strafe)
      xSpeed = autoXSpeed;

      SmartDashboard.putBoolean("FaceTags/AutoDriving", true);
      SmartDashboard.putNumber("FaceTags/AutoXSpeed", autoXSpeed);
    } else {
      SmartDashboard.putBoolean("FaceTags/AutoDriving", false);
    }

    // Limit rotation speed
    rotationSpeed = Math.max(-4.0, Math.min(4.0, rotationSpeed));

    // Apply field-oriented control with auto-rotation
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotationSpeed, currentPose.getRotation());

    drive.runVelocity(fieldRelativeSpeeds);

    // Logging
    SmartDashboard.putNumber("FaceTags/CurrentDistance", currentDistance);
    SmartDashboard.putNumber("FaceTags/TargetDistance", AutoAlign.HUB_DISTANCE_METERS);
    SmartDashboard.putNumber("FaceTags/DesiredHeading", desiredHeading.getDegrees());
    SmartDashboard.putNumber(
        "FaceTags/CurrentHeading", currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber(
        "FaceTags/HeadingError",
        desiredHeading.minus(currentPose.getRotation()).getDegrees());
    SmartDashboard.putNumber("FaceTags/RotationSpeed", rotationSpeed);
    SmartDashboard.putString("FaceTags/VisibleTags", visibleTags.toString());
    SmartDashboard.putBoolean(
        "FaceTags/AtTargetDistance",
        Math.abs(currentDistance - AutoAlign.HUB_DISTANCE_METERS)
            < AutoAlign.DISTANCE_TOLERANCE_METERS);
    SmartDashboard.putBoolean(
        "FaceTags/AtTargetHeading", rotationController.atSetpoint());
  }

  /**
   * Get list of visible tags from both cameras.
   *
   * @return List of tag IDs that are currently visible and in our target list
   */
  private List<Integer> getVisibleTags() {
    List<Integer> visibleTags = new ArrayList<>();

    for (int tagId : currentTagList) {
      boolean frontSeesTag =
          vision.frontCameraHasTargets() && vision.getFrontCameraBestTargetId() == tagId;
      boolean backSeesTag =
          vision.backCameraHasTargets() && vision.getBackCameraBestTargetId() == tagId;

      if (frontSeesTag || backSeesTag) {
        visibleTags.add(tagId);
      }
    }

    return visibleTags;
  }

  /**
   * Calculate vision-corrected midpoint between visible tags using actual vision measurements.
   *
   * @param visibleTags List of currently visible tag IDs
   * @return Vision-corrected midpoint, or null if no correction available
   */
  private Translation2d calculateVisionCorrectedMidpoint(List<Integer> visibleTags) {
    List<Translation2d> tagPositions = new ArrayList<>();

    for (int tagId : visibleTags) {
      Optional<Translation2d> tagPos = getTagPositionFromVision(tagId);
      if (tagPos.isPresent()) {
        tagPositions.add(tagPos.get());
      }
    }

    if (tagPositions.isEmpty()) {
      return null;
    }

    return calculateMidpoint(tagPositions);
  }

  /**
   * Calculate midpoint from a list of positions.
   *
   * @param positions List of 2D positions
   * @return Midpoint (average) of all positions
   */
  private Translation2d calculateMidpoint(List<Translation2d> positions) {
    if (positions.isEmpty()) {
      return new Translation2d();
    }

    double sumX = 0.0;
    double sumY = 0.0;

    for (Translation2d pos : positions) {
      sumX += pos.getX();
      sumY += pos.getY();
    }

    return new Translation2d(sumX / positions.size(), sumY / positions.size());
  }

  /**
   * Get tag position from vision measurements.
   *
   * @param tagId The AprilTag ID
   * @return Optional containing tag position from field layout
   */
  private Optional<Translation2d> getTagPositionFromVision(int tagId) {
    // Check if either camera sees this tag
    boolean frontSeesTag =
        vision.frontCameraHasTargets() && vision.getFrontCameraBestTargetId() == tagId;
    boolean backSeesTag =
        vision.backCameraHasTargets() && vision.getBackCameraBestTargetId() == tagId;

    if (!frontSeesTag && !backSeesTag) {
      return Optional.empty();
    }

    // Use field layout position (already corrected by vision fusion in Drive subsystem)
    var tagPose = vision.getFieldLayout().getTagPose(tagId);
    return tagPose.map(pose3d -> pose3d.toPose2d().getTranslation());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    SmartDashboard.putString(
        "FaceTags/Status", interrupted ? "INTERRUPTED" : "FINISHED");
  }

  @Override
  public boolean isFinished() {
    // Command runs while button is held
    return false;
  }
}