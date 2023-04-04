package frc.robot.commands.auto.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.team_8840_lib.info.console.Logger;

public class StopArmMovement extends CommandBase {
    public StopArmMovement() {

    }

    @Override
    public void initialize() {
        ArmSubsystem.getInstance().stop();
        Logger.Log("Arm", "Force stopped arm movement.");
    }

    @Override
    public void execute() {
        //Do Nothing.
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
