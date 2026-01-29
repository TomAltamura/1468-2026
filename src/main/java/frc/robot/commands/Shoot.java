package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class Shoot extends Command {
  private final ShooterSubsystem shooter;
  private final DoubleSupplier flywheelSpeed;

  /**
   * Creates a Shoot command.
   *
   * @param shooter The subsystem used by this command.
   * @param flywheelSpeed Supplier for flywheel speed (-1.0 to 1.0).
   */
  public Shoot(ShooterSubsystem shooter, DoubleSupplier flywheelSpeed) {
    this.shooter = shooter;
    this.flywheelSpeed = flywheelSpeed;

    addRequirements(shooter);
  }

  @Override
  public void execute() {
    double speed = flywheelSpeed.getAsDouble();
    if (speed < 0.2 && speed > -0.2) {
      shooter.setFlywheel(0.0);
    } else {
      shooter.setFlywheel(speed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
