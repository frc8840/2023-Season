package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team_8840_lib.info.console.Logger;

public class PlacePiece extends CommandBase {
    public PlacePiece() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    @Override
    public void initialize() {
        //etc
        Logger.Log("[PlacePiece] PlacePiece initialized.");
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
