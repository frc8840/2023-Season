package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team_8840_lib.info.console.Logger;

public class IntakeCommand extends CommandBase {
    public IntakeCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Logger.Log("[IntakeCommand] IntakeCommand initialized");
    }

    @Override
    public void execute() {
        //etc
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
