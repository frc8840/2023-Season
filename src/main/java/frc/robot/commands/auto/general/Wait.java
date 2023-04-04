package frc.robot.commands.auto.general;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team_8840_lib.info.console.Logger;

public class Wait extends CommandBase {
    private double ms;

    public Wait(double time) {
        ms = time;
    }

    private double startTime;

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        Logger.Log("Wait", "Waiting for " + ms + "ms.");
    }

    @Override
    public void execute() {
        //Do nothing
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= ms;
    }
}
