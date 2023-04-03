package frc.robot.commands.auto.movement;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.utils.ModuleConstants;
import frc.team_8840_lib.utils.controllers.Pigeon;

public class RotateTo extends CommandBase {
    private Rotation2d target;
    private Rotation2d start;
    private Rotation2d leniency;

    public RotateTo(Rotation2d target, Rotation2d leniency) {
        this.target = target;

        if (leniency.getDegrees() < 0) {
            throw new IllegalArgumentException("Leniency must be positive!");
        }

        if (leniency.getDegrees() > 45) {
            throw new IllegalArgumentException("Leniency must be less than 45 degrees!");
        }

        this.leniency = leniency;
    }
    
    @Override
    public void initialize() {
        start = Rotation2d.fromDegrees(Pigeon.getPigeon(ModuleConstants.PIGEON_ID).getYawPitchRoll()[0]);
    }

    @Override
    public void execute() {
        Rotation2d currentAngle = Rotation2d.fromDegrees(Pigeon.getPigeon(ModuleConstants.PIGEON_ID).getYawPitchRoll()[0]);

        double angle = target.getDegrees() - currentAngle.getDegrees();
        double totalRotation = target.getDegrees() - start.getDegrees();

        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }

        if (totalRotation > 180) {
            totalRotation -= 360;
        } else if (totalRotation < -180) {
            totalRotation += 360;
        }

        if (Math.abs(angle) < leniency.getDegrees()) {
            RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().stop();
        } else {
            RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().spin(
                (angle / 180) * Math.PI * (Math.max(0.1, 1 - Math.abs(angle / totalRotation)))
            );
        }
    }

    @Override
    public boolean isFinished() {
        double currentAngle = Pigeon.getPigeon(ModuleConstants.PIGEON_ID).getYawPitchRoll()[0];

        //Return true if the angle is within the leniency, or if it's overshot.
        boolean inLenciency = Math.abs(target.getDegrees() - currentAngle) < leniency.getDegrees();
        boolean overshot = Math.abs(target.getDegrees() - start.getDegrees()) < Math.abs(target.getDegrees() - currentAngle);

        return inLenciency || overshot;
    }
}
