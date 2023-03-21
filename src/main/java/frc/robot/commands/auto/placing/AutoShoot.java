package frc.robot.commands.auto.placing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.GrabberSubsystem;
import frc.team_8840_lib.info.console.Logger;

public class AutoShoot extends CommandBase {
    public enum ShootType {
        HYBRID(0.1),
        MID(0.5),
        HIGH(0.7);

        private double speed;

        private ShootType(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return speed;
        }
    }

    private static final double runTime = 1000;

    private ShootType type;

    private double startTime;

    public AutoShoot(ShootType type) {
        this.type = type;
    }

    @Override
    public void initialize() {
        Logger.Log("[AutoShoot] Started AutoShoot!");
        
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double now = System.currentTimeMillis();

        if (now - startTime < runTime) {
            GrabberSubsystem.getInstance().outtake(this.type.speed);
        }
    }
}
