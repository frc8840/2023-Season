package frc.robot.commands;

import java.util.Date;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Measurements;
import frc.team_8840_lib.input.communication.CommunicationManager;
import frc.team_8840_lib.utils.controllers.Pigeon;

public class BalanceCommand extends CommandBase {
    //TODO: Tune these values to the max pitch or roll of the robot when it's on the balance beam.
    private static final double maxRotation = 0; //in the unit of the gyro
    private static final double maxSpeed = Measurements.DockingStation.WIDTH; //in m/s

    private BalanceReference reference = BalanceReference.Pitch;

    private PIDController pid;

    private double lastPIDOutput = 0;

    /**
     * Creates a new Balance Subsystem.
     */
    public BalanceCommand() {
        //TODO: Tune PID controller.
        pid = new PIDController(
            0.1, 0, 0
        );

        //Set the reference so the comms can be updated.
        setReference(BalanceReference.Pitch);

        //Reset everything.
        reset();
    }

    /**
     * Reset the PID and set the last PID output to 0.
     */
    public void reset() {
        pid.reset();
        lastPIDOutput = 0;
    }

    /**
     * Set whether the robot should use either pitch or roll to balance.
     * @param reference Pitch or roll
     */
    public void setReference(BalanceReference reference) {
        this.reference = reference;

        //We'll set the reference to the NT server so that the dashboard can use it.
        //This can also be updated by the dashboard.
        CommunicationManager.getInstance()
            .updateInfo("systems", "balancing/axis", this.reference.name())
        //We'll also get the time to update to NT. This is useful for checking that the change was successful.
            .updateInfo("systems", "balancing/axis_confirm", (new Date()).getTime());
    }

    /*
     * Get the current gyro value, based on the reference.
     */
    public double getGyroValue() {
        //Get the [yaw pitch roll]. If the pigeon is null, return 0.
        double[] ypr = Pigeon.hasPigeon(0) != null ? Pigeon.getPigeon(0).getYawPitchRoll() : new double[] {0, 0, 0};

        switch (this.reference) {
            case Roll:
                return ypr[2];
            case Pitch:
                return ypr[1];
            default:
                return 0;
        }
    }

    @Override
    public void execute() {
        //Get the current gyro angle.

        lastPIDOutput = pid.calculate(getGyroValue());

        //TODO: Figure out how to use the PID value to go towards the center of the beam.
    }

    //Related to the subsystem
    public static enum BalanceReference {
        Roll, Pitch; //Yaw isn't included since that's not relevant to balancing.
    }
}
