// THIS IS AI GENERATED. WE REALLY NEED TO TEST THIS TO SEE HOW IT WORKS WITH THE APRIL TAGS
// I SUGGEST PUTTING THIS ON 2 by 4 for testing and then set it on the ground
//
// ============================================================================
// COMMAND OVERVIEW: Auto-Aim at Hub
// ============================================================================
// WHAT THIS DOES:
// 1. AUTO-AIMS: Continuously rotates robot to face your alliance hub (driver controls XY movement
// only)
// 2. AUTO-DRIVES: When you see hub tags AND are within range, automatically drives to maintain
// target distance
// 3. USES ODOMETRY: Trusts robot position to aim at hub coordinates
// 4. HUB TAGS OPTIONAL: Only needed to trigger auto-drive; will still aim at hub without seeing
// them
//

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
 * Command that auto-rotates robot to face hub coordinates using trusted odometry. - Driver controls
 * XY translation (field-oriented) - Robot ALWAYS rotates to face hub using odometry (already fused
 * with vision) - Auto-drives to target distance ONLY when within threshold and seeing hub tags
 */
public class FaceTagsCommand extends Command {
  private final Drive drive;
  private final VisionSubsystem vision;
  private final DoubleSupplier xSpeedSupplier;
  private final DoubleSupplier ySpeedSupplier;

  // Hub coordinates (middle of the hub)
  private static final Translation2d RED_HUB_POSITION =
      new Translation2d(Units.inchesToMeters(468.57), Units.inchesToMeters(158.32));
  private static final Translation2d BLUE_HUB_POSITION =
      new Translation2d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.32));

  // Target AprilTag IDs (for vision-based auto-drive trigger only)
  private static final List<Integer> RED_HUB_TAGS = List.of(25, 26);
  private static final List<Integer> BLUE_HUB_TAGS = List.of(9, 10);

  // PID Controllers
  private final PIDController rotationController =
      new PIDController(AutoAlign.ROTATION_kP, AutoAlign.ROTATION_kI, AutoAlign.ROTATION_kD);
  private final PIDController distanceController =
      new PIDController(AutoAlign.DISTANCE_kP, AutoAlign.DISTANCE_kI, AutoAlign.DISTANCE_kD);

  // Target tracking
  private Translation2d hubPosition;
  private List<Integer> currentTagList;
  private boolean hasInitialized = false;

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

    // Determine which hub to target based on alliance
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      SmartDashboard.putString("FaceTags/Status", "NO ALLIANCE");
      cancel();
      return;
    }

    // Select hub position and tag list based on alliance
    if (alliance.get() == Alliance.Red) {
      hubPosition = RED_HUB_POSITION;
      currentTagList = RED_HUB_TAGS;
    } else {
      hubPosition = BLUE_HUB_POSITION;
      currentTagList = BLUE_HUB_TAGS;
    }

    // Reset controllers
    rotationController.reset();
    distanceController.reset();

    hasInitialized = true;
    SmartDashboard.putString("FaceTags/Status", "ACTIVE");
    SmartDashboard.putString("FaceTags/Alliance", alliance.get().toString());
    SmartDashboard.putString(
        "FaceTags/HubPosition",
        String.format("X=%.2fm Y=%.2fm", hubPosition.getX(), hubPosition.getY()));
  }

  @Override
  public void execute() {
    if (!hasInitialized) return;

    // Get current robot pose from odometry (ALREADY vision-fused by Drive subsystem!)
    Pose2d currentPose = drive.getPose();
    Translation2d robotPosition = currentPose.getTranslation();

    // Calculate direction to hub
    Translation2d robotToHub = hubPosition.minus(robotPosition);
    double currentDistance = robotToHub.getNorm();
    Rotation2d desiredHeading = robotToHub.getAngle();

    // Calculate rotation speed to face hub
    double rotationSpeed =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), desiredHeading.getRadians());

    // Get driver translation inputs (field-oriented)
    double xSpeed = xSpeedSupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec();
    double ySpeed = ySpeedSupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec();

    // Check if we can see hub tags for auto-drive
    List<Integer> visibleHubTags = getVisibleHubTags();
    boolean seesHubTags = !visibleHubTags.isEmpty();

    // Auto-drive to target distance if:
    // 1. We see at least one hub tag
    // 2. We're within distance threshold
    // 3. Driver isn't providing forward/back input
    boolean shouldAutoDrive =
        seesHubTags
            && currentDistance < AutoAlign.VISION_DISTANCE_THRESHOLD_METERS
            && Math.abs(xSpeed) < 0.1;

    if (shouldAutoDrive) {
      // Override X speed to maintain target distance
      double distanceError = currentDistance - AutoAlign.HUB_DISTANCE_METERS;
      double autoXSpeed = distanceController.calculate(0, -distanceError);

      // Limit auto speed
      autoXSpeed = Math.max(-2.0, Math.min(2.0, autoXSpeed));

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
    SmartDashboard.putNumber("FaceTags/CurrentHeading", currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber(
        "FaceTags/HeadingError", desiredHeading.minus(currentPose.getRotation()).getDegrees());
    SmartDashboard.putNumber("FaceTags/RotationSpeed", rotationSpeed);
    SmartDashboard.putString("FaceTags/VisibleHubTags", visibleHubTags.toString());
    SmartDashboard.putBoolean("FaceTags/SeesHubTags", seesHubTags);
    SmartDashboard.putBoolean(
        "FaceTags/WithinAutoDriveRange",
        currentDistance < AutoAlign.VISION_DISTANCE_THRESHOLD_METERS);
    SmartDashboard.putBoolean(
        "FaceTags/AtTargetDistance",
        Math.abs(currentDistance - AutoAlign.HUB_DISTANCE_METERS)
            < AutoAlign.DISTANCE_TOLERANCE_METERS);
    SmartDashboard.putBoolean("FaceTags/AtTargetHeading", rotationController.atSetpoint());
  }

  /** Get list of visible hub tags (for auto-drive trigger only). */
  private List<Integer> getVisibleHubTags() {
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

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    SmartDashboard.putString("FaceTags/Status", interrupted ? "INTERRUPTED" : "FINISHED");
  }

  @Override
  public boolean isFinished() {
    return false; // Runs while button held
  }
}
