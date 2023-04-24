package frc.robot.commands.auto.movement;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.utils.ModuleConstants;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.utils.controllers.Pigeon;

public class PIDRotate extends CommandBase {
    private Rotation2d target;
    private Rotation2d start;
    private Rotation2d leniency;

    public static PIDController pid = new PIDController(0.2, 0, 0);

    private double maxDegreesPerSecond = 90;

    public PIDRotate(Rotation2d target, Rotation2d leniency) {
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
        Logger.Log("PIDRotate", "Starting PIDRotate to " + target.getDegrees() + " degrees.");
    }

    @Override
    public void execute() {
        //Use the PID controller to calculate the output
        Rotation2d currentAngle = Rotation2d.fromDegrees(Pigeon.getPigeon(ModuleConstants.PIGEON_ID).getYawPitchRoll()[0]);

        double output = pid.calculate(currentAngle.getDegrees(), target.getDegrees());

        SmartDashboard.putNumber("Rotation Movement", output);

        //Limit the output to the max degrees per second
        output = Math.min(output, Math.abs(maxDegreesPerSecond)) * Math.signum(output);

        //Convert the output to radians per second
        output = (output / 180) * Math.PI;

        //Stop the robot if the angle is within the leniency
        if (Math.abs(target.getDegrees() - currentAngle.getDegrees()) < leniency.getDegrees()) {
            RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().stop();
        } else {
            RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().spin(output);
        }
    }

    @Override
    public boolean isFinished() {
        double currentAngle = Pigeon.getPigeon(ModuleConstants.PIGEON_ID).getYawPitchRoll()[0];

        //Return true if the angle is within the leniency, or if it's overshot.
        boolean inLenciency = Math.abs(target.getDegrees() - currentAngle) < leniency.getDegrees();
        //boolean overshot = Math.abs(target.getDegrees() - start.getDegrees()) < Math.abs(target.getDegrees() - currentAngle);

        return inLenciency; //|| overshot;
    }
}
