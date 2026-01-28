package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.harvester.HarvesterSubsystem;
import frc.robot.Constants;

public class HarvesterSpin extends Command {
    private final HarvesterSubsystem harvester;

    public HarvesterSpin(HarvesterSubsystem harvester) {
        this.harvester = harvester;
        addRequirements(harvester);
    }

    @Override
    public void execute() {
        harvester.setSpinVelocity(Constants.Harvester.SPIN_TARGET_RPS);
    }

    @Override
    public void end(boolean interrupted) {
        harvester.stopSpin();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}