//HAVE TO BIND TO BUTTON
//NEED TO ADD INTAKE SYSTEM TOO
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Kicker;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSequence extends Command {
  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final double targetVelocityRPS;
  private boolean kickerStarted = false;

  /**
   * Creates a command that spins up the shooter, waits for velocity, then starts the kicker.
   *
   * @param shooter The shooter subsystem
   * @param kicker The kicker subsystem
   * @param targetVelocityRPS Target shooter velocity in RPS
   */
  public ShootSequence(
      ShooterSubsystem shooter, KickerSubsystem kicker, double targetVelocityRPS) {
    this.shooter = shooter;
    this.kicker = kicker;
    this.targetVelocityRPS = targetVelocityRPS;
    addRequirements(shooter, kicker);
  }

  @Override
  public void initialize() {
    kickerStarted = false;
    shooter.setFlywheel(targetVelocityRPS);
  }

  @Override
  public void execute() {
    // Once shooter reaches velocity and kicker hasn't started yet, start the kicker
    if (!kickerStarted && shooter.isAtVelocity()) {
      kicker.setVelocity(Kicker.KICKER_TARGET_RPS);
      kickerStarted = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    kicker.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }
}