package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.GrabberSubsystem;
import frc.robot.subsystems.intake.GrabberSubsystem.LoadedPiece;
import frc.team_8840_lib.info.console.Logger;

public class IntakeCommand extends CommandBase {

    private double time;

    public IntakeCommand(double time) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.time = time;
    }

    private double startTime = 0;

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        Logger.Log("[IntakeCommand] IntakeCommand initialized");
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() - startTime < time) {
            GrabberSubsystem.getInstance().intake(LoadedPiece.CUBE);
        } else {
            GrabberSubsystem.getInstance().stopIntake();
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= time;
    }
}
