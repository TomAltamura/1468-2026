package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.harvester.HarvesterSubsystem;
import frc.robot.Constants;

public class HarvesterDeploy extends Command {
    private final HarvesterSubsystem harvester;

    public HarvesterDeploy(HarvesterSubsystem harvester) {
        this.harvester = harvester;
        addRequirements(harvester);
    }

    @Override
    public void execute() {
        harvester.setDeployPosition(Constants.Harvester.DEPLOY_TARGET_POSITION);
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