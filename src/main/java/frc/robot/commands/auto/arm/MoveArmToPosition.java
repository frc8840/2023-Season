package frc.robot.commands.auto.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ArmState;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.listeners.Robot;

public class MoveArmToPosition extends CommandBase {
    private ArmState state;

    public MoveArmToPosition(ArmState state) {
        this.state = state;
    }

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().setPosition(state);
        Logger.Log("Arm", "Moving arm to " + state.name());
    }

    @Override
    public void execute() {
        //Do Nothing.
    }

    @Override
    public boolean isFinished() {
        return ArmSubsystem.getInstance().isAtPosition(state, Rotation2d.fromDegrees(8)) || Robot.isSimulation();
    }
}
