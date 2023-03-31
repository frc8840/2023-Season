package frc.robot.commands.auto.movement;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.utils.ModuleConstants;
import frc.team_8840_lib.utils.controllers.Pigeon;

public class RotateTo extends CommandBase {
    private Rotation2d target;
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
        
    }

    @Override
    public void execute() {
        Rotation2d currentAngle = Rotation2d.fromDegrees(Pigeon.getPigeon(ModuleConstants.PIGEON_ID).getYawPitchRoll()[0]);

        double angle = target.getDegrees() - currentAngle.getDegrees();

        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }

        if (Math.abs(angle) < leniency.getDegrees()) {
            RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().stop();
        } else {
            RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().spin(angle / 180);
        }
    }

    @Override
    public boolean isFinished() {
        double angle = target.getDegrees() - Pigeon.getPigeon(ModuleConstants.PIGEON_ID).getYawPitchRoll()[0];

        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }

        return Math.abs(angle) < leniency.getDegrees();
    }
}
