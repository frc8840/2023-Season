package frc.robot.commands.auto.movement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class StopDrive extends CommandBase {
    public StopDrive() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().stop();
    }

    @Override
    public void execute() {
        RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
