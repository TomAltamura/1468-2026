package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Harvester;
import frc.robot.subsystems.HarvesterSubsystem;

public class HarvesterDeploy extends Command {
  private final HarvesterSubsystem harvester;

  public HarvesterDeploy(HarvesterSubsystem harvester) {
    this.harvester = harvester;
    addRequirements(harvester);
  }

  @Override
  public void execute() {
    harvester.setDeployPosition(Harvester.DEPLOY_TARGET_POSITION);
  }

  @Override
  public void end(boolean interrupted) {
    harvester.stopDeploy();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
