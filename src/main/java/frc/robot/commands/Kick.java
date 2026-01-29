package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Kicker;
import frc.robot.subsystems.KickerSubsystem;

public class Kick extends Command {
  private final KickerSubsystem kicker;

  public Kick(KickerSubsystem kicker) {
    this.kicker = kicker;
    addRequirements(kicker);
  }

  @Override
  public void execute() {
    kicker.setVelocity(Kicker.KICKER_TARGET_RPS); // Set to 30 RPS
  }

  @Override
  public void end(boolean interrupted) {
    kicker.stop();
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }
}
