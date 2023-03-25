package frc.robot.commands.auto.movement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SmallRotate extends CommandBase {
    public SmallRotate() {
        
    }

    private double startTime = 0;

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double now = System.currentTimeMillis();
        if (now - startTime < 200) {
            //Rotate the robot by 0.1 radians per second
            RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().spin(0.1);
        } else {
            RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().stop();
        }
    }

    @Override
    public boolean isFinished() {
        double now = System.currentTimeMillis();
        return now - startTime > 200;
    }
}
