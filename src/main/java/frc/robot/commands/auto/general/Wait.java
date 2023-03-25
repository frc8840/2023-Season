package frc.robot.commands.auto.general;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
    private double ms;

    public Wait(double time) {
        ms = time;
    }

    private double startTime;

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
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
