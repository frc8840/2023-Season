package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmControl extends CommandBase {
    public static enum ArmState {
        RESTING(
            Rotation2d.fromDegrees(0), 
            Rotation2d.fromDegrees(0)
        ),
        INTAKE_FLOOR(
            Rotation2d.fromDegrees(0), 
            Rotation2d.fromDegrees(0)
        ),
        INTAKE_SUBSTATION(
            Rotation2d.fromDegrees(0), 
            Rotation2d.fromDegrees(0)
        ),
        PLACING_HYBRID(
            Rotation2d.fromDegrees(0), 
            Rotation2d.fromDegrees(0)
        ),
        PLACING_MIDDLE_CONE(
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(0)
        ),
        PLACING_MIDDLE_CUBE(
            Rotation2d.fromDegrees(0), 
            Rotation2d.fromDegrees(0)
        ),
        PLACING_UPPER_CONE(
            Rotation2d.fromDegrees(0), 
            Rotation2d.fromDegrees(0)
        ),
        PLACING_UPPER_CUBE(
            Rotation2d.fromDegrees(0), 
            Rotation2d.fromDegrees(0)
        );

        private Rotation2d baseAngle;
        private Rotation2d elbowAngle;

        private ArmState(Rotation2d baseAngle, Rotation2d elbowAngle) {
            this.baseAngle = baseAngle;
            this.elbowAngle = elbowAngle;
        }
    }
    
    private ArmState state;
    private boolean requestUpdateAngle = false;

    private ArmSubsystem arm;

    public ArmControl(ArmSubsystem arm) {
        state = ArmState.RESTING;

        this.arm = arm;

        addRequirements(arm);
    }

    public void setState(ArmState state) {
        this.state = state;
        requestUpdateAngle = true;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (requestUpdateAngle) {
            requestUpdateAngle = false;

            this.arm.setPosition(state.baseAngle, state.elbowAngle);
        }
    }
 
    @Override
    public boolean isFinished() {
        return false;
    }
}
