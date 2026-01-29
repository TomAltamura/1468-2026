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
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * Command that auto-rotates robot to face processor AprilTags. - Driver controls XY translation
 * (field-oriented) - Robot auto-rotates to face midpoint between tags - If vision sees tags,
 * auto-drives to 10 feet away - Uses real-time vision feedback to correct odometry drift
 */
public class FaceTagsCommand extends Command {
  private final Drive drive;
  private final VisionSubsystem vision;
  private final DoubleSupplier xSpeedSupplier;
  private final DoubleSupplier ySpeedSupplier;

  // Target AprilTag IDs (processor station tags)
  private static final int BLUE_TAG_1 = 9;
  private static final int BLUE_TAG_2 = 10;
  private static final int RED_TAG_1 = 25;
  private static final int RED_TAG_2 = 26;

  // Distance control
  private static final double TARGET_DISTANCE = Units.feetToMeters(10.0); // 10 feet
  private static final double DISTANCE_TOLERANCE = 0.3; // meters
  private static final double VISION_DISTANCE_THRESHOLD =
      Units.feetToMeters(15.0); // Only auto-drive if within 15ft

  // PID Controllers
  private final PIDController rotationController = new PIDController(5.0, 0.0, 0.2);
  private final PIDController distanceController = new PIDController(2.0, 0.0, 0.1);

  // Target tracking
  private Translation2d targetMidpoint;
  private int tag1Id, tag2Id;
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
    rotationController.setTolerance(Units.degreesToRadians(2.0));
  }

  @Override
  public void initialize() {
    hasInitialized = false;

    // Determine which tags to target based on alliance
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      SmartDashboard.putString("FaceProcessorTags/Status", "NO ALLIANCE");
      cancel();
      return;
    }

    if (alliance.get() == Alliance.Blue) {
      tag1Id = BLUE_TAG_1;
      tag2Id = BLUE_TAG_2;
    } else {
      tag1Id = RED_TAG_1;
      tag2Id = RED_TAG_2;
    }

    // Get tag positions from field layout
    var tag1Pose = vision.getFieldLayout().getTagPose(tag1Id);
    var tag2Pose = vision.getFieldLayout().getTagPose(tag2Id);

    if (tag1Pose.isEmpty() || tag2Pose.isEmpty()) {
      SmartDashboard.putString("FaceProcessorTags/Status", "TAGS NOT IN LAYOUT");
      cancel();
      return;
    }

    // Calculate midpoint between the two tags
    Translation2d tag1Pos = tag1Pose.get().toPose2d().getTranslation();
    Translation2d tag2Pos = tag2Pose.get().toPose2d().getTranslation();
    targetMidpoint = tag1Pos.plus(tag2Pos).div(2.0);

    // Reset controllers
    rotationController.reset();
    distanceController.reset();

    hasInitialized = true;
    SmartDashboard.putString("FaceProcessorTags/Status", "ACTIVE");
    SmartDashboard.putNumber("FaceProcessorTags/Tag1ID", tag1Id);
    SmartDashboard.putNumber("FaceProcessorTags/Tag2ID", tag2Id);
    SmartDashboard.putString(
        "FaceProcessorTags/TargetMidpoint",
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

    // Check if we can see either target tag
    boolean seesTag1 =
        (vision.frontCameraHasTargets() && vision.getFrontCameraBestTargetId() == tag1Id)
            || (vision.backCameraHasTargets() && vision.getBackCameraBestTargetId() == tag1Id);
    boolean seesTag2 =
        (vision.frontCameraHasTargets() && vision.getFrontCameraBestTargetId() == tag2Id)
            || (vision.backCameraHasTargets() && vision.getBackCameraBestTargetId() == tag2Id);
    boolean seesAnyTag = seesTag1 || seesTag2;

    // Update target midpoint from vision if we see tags (real-time correction)
    if (seesAnyTag) {
      Translation2d visionCorrectedMidpoint = calculateVisionCorrectedMidpoint();
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
        seesAnyTag && currentDistance < VISION_DISTANCE_THRESHOLD && Math.abs(xSpeed) < 0.1;

    if (shouldAutoDrive) {
      // Override X speed to maintain target distance
      double distanceError = currentDistance - TARGET_DISTANCE;
      double autoXSpeed = distanceController.calculate(0, -distanceError);

      // Limit auto speed
      autoXSpeed = Math.max(-2.0, Math.min(2.0, autoXSpeed));

      // Blend with driver input (driver can still strafe)
      xSpeed = autoXSpeed;

      SmartDashboard.putBoolean("FaceProcessorTags/AutoDriving", true);
      SmartDashboard.putNumber("FaceProcessorTags/AutoXSpeed", autoXSpeed);
    } else {
      SmartDashboard.putBoolean("FaceProcessorTags/AutoDriving", false);
    }

    // Limit rotation speed
    rotationSpeed = Math.max(-4.0, Math.min(4.0, rotationSpeed));

    // Apply field-oriented control with auto-rotation
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotationSpeed, currentPose.getRotation());

    drive.runVelocity(fieldRelativeSpeeds);

    // Logging
    SmartDashboard.putNumber("FaceProcessorTags/CurrentDistance", currentDistance);
    SmartDashboard.putNumber("FaceProcessorTags/TargetDistance", TARGET_DISTANCE);
    SmartDashboard.putNumber("FaceProcessorTags/DesiredHeading", desiredHeading.getDegrees());
    SmartDashboard.putNumber(
        "FaceProcessorTags/CurrentHeading", currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber(
        "FaceProcessorTags/HeadingError",
        desiredHeading.minus(currentPose.getRotation()).getDegrees());
    SmartDashboard.putNumber("FaceProcessorTags/RotationSpeed", rotationSpeed);
    SmartDashboard.putBoolean("FaceProcessorTags/SeesTag1", seesTag1);
    SmartDashboard.putBoolean("FaceProcessorTags/SeesTag2", seesTag2);
    SmartDashboard.putBoolean(
        "FaceProcessorTags/AtTargetDistance",
        Math.abs(currentDistance - TARGET_DISTANCE) < DISTANCE_TOLERANCE);
    SmartDashboard.putBoolean("FaceProcessorTags/AtTargetHeading", rotationController.atSetpoint());
  }

  /**
   * Calculate vision-corrected midpoint between tags using actual vision measurements. This
   * provides real-time feedback to correct odometry drift.
   */
  private Translation2d calculateVisionCorrectedMidpoint() {
    // Get the latest vision pose estimates that include the target tags
    Optional<Translation2d> tag1PosFromVision = getTagPositionFromVision(tag1Id);
    Optional<Translation2d> tag2PosFromVision = getTagPositionFromVision(tag2Id);

    // If we see both tags, use their actual positions
    if (tag1PosFromVision.isPresent() && tag2PosFromVision.isPresent()) {
      return tag1PosFromVision.get().plus(tag2PosFromVision.get()).div(2.0);
    }

    // If we see one tag, offset from its position
    if (tag1PosFromVision.isPresent()) {
      // Estimate tag2 position based on known offset
      Translation2d knownOffset = getKnownTagOffset();
      return tag1PosFromVision.get().plus(knownOffset.div(2.0));
    }

    if (tag2PosFromVision.isPresent()) {
      Translation2d knownOffset = getKnownTagOffset();
      return tag2PosFromVision.get().minus(knownOffset.div(2.0));
    }

    return null; // No vision correction available
  }

  /**
   * Get tag position from vision measurements by combining: - Current robot pose estimate (from
   * odometry + vision fusion) - Known tag position in field layout
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

    // Use field layout position (this is already corrected by vision fusion in Drive subsystem)
    var tagPose = vision.getFieldLayout().getTagPose(tagId);
    return tagPose.map(pose3d -> pose3d.toPose2d().getTranslation());
  }

  /** Get known offset between the two processor tags. */
  private Translation2d getKnownTagOffset() {
    var tag1Pose = vision.getFieldLayout().getTagPose(tag1Id);
    var tag2Pose = vision.getFieldLayout().getTagPose(tag2Id);

    if (tag1Pose.isPresent() && tag2Pose.isPresent()) {
      return tag2Pose
          .get()
          .toPose2d()
          .getTranslation()
          .minus(tag1Pose.get().toPose2d().getTranslation());
    }

    return new Translation2d(); // Zero offset if unknown
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    SmartDashboard.putString("FaceProcessorTags/Status", interrupted ? "INTERRUPTED" : "FINISHED");
  }

  @Override
  public boolean isFinished() {
    // Command runs while button is held
    return false;
  }
}
