package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.utils.controllers.Pigeon;

public class AutoBalance extends CommandBase {
    public static enum MoveState { 
        TOWARDS_CHARGE_STATION,
        BALANCE,
        IDLE;
    }

    private final double levelDegree = 6;
    private final double chargeStationDegree = 13;

    private final double fastSpeed = 3; //m/s
    private final double slowSpeed = 1; //m/s

    private double secondsToTicks(double seconds) {
        return seconds / Robot.DELTA_TIME;
    }

    private MoveState state = MoveState.TOWARDS_CHARGE_STATION;
    private double tickCounter = 0;

    public AutoBalance(MoveState startState) {
        state = startState;
        tickCounter = 0;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    public double getTilt() {
        double[] yaw_pitch_roll = Pigeon.getPigeon(42).getYawPitchRoll();
        double pitch = yaw_pitch_roll[1];
        double roll = yaw_pitch_roll[2];

        if (pitch + roll >= 0) {
            return Math.sqrt(Math.pow(pitch, 2) + Math.pow(roll, 2));
        } else {
            return -Math.sqrt(Math.pow(pitch, 2) + Math.pow(roll, 2));
        }
    }

    @Override
    public void execute() {
        //TODO: TEST THIS CODE.
        DriveSubsystem drive = RobotContainer.getInstance().getDriveSubsystem();

        if (state == MoveState.TOWARDS_CHARGE_STATION) {
            drive.getSwerveDrive().drive(
                new Translation2d(
                    fastSpeed, 
                    0
                ), 
                0, 
                true, 
                Robot.isReal()
            );

            if (secondsToTicks(1) <= tickCounter) {
                state = MoveState.BALANCE;
                tickCounter = 0;
            }
        } else if (state == MoveState.BALANCE) {
            double tilt = getTilt();

            double speed = 0;

            if (tilt > levelDegree) {
                speed = slowSpeed;
            } else if (tilt < levelDegree) {
                speed = -slowSpeed;
            }

            drive.getSwerveDrive().drive(
                new Translation2d(speed, 0), 
                0, 
                true, 
                Robot.isReal()
            );
        }
        
        tickCounter++;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
