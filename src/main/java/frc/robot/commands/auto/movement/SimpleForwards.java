package frc.robot.commands.auto.movement;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.listeners.Robot;

public class SimpleForwards extends CommandBase {
    private double ms;

    public SimpleForwards(double time) { ms = time; }

    private double startTime;

    @Override
    public void initialize() {
        Logger.Log("[SimpleForwards] Started SimpleForwards!");
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double now = System.currentTimeMillis();

        if (now - startTime < ms) {
            Translation2d movement = new Translation2d(
                -2,
                0
            );

            RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().drive(
                movement, 0, true, Robot.isReal()
            );
        } else {
            RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().stop();
        }
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= ms;
    }


}
