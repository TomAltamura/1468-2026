package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Indexer;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerSpin extends Command {
  private final IndexerSubsystem indexer;

  public IndexerSpin(IndexerSubsystem indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void execute() {
    indexer.setVelocity(Indexer.TARGET_RPS);
  }

  @Override
  public void end(boolean interrupted) {

    // Find nearest position
    double currentPosition = indexer.getPosition();
    double nearest = Indexer.POSITIONS[0];
    double minDiff = Math.abs(currentPosition - nearest);

    for (double pos : Indexer.POSITIONS) {
      double diff = Math.abs(currentPosition - pos);
      if (diff < minDiff) {
        minDiff = diff;
        nearest = pos;
      }
    }
    indexer.setPosition(nearest);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
