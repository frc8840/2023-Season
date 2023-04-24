package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.utils.controllers.Pigeon;

public class AutoBalance extends CommandBase {
    public static enum MoveState { 
        TOWARDS_CHARGE_STATION,
        SLOW_UP,
        BALANCE,
        IDLE;
    }

    private final double levelDegree = 6;
    private final double chargeStationDegree = 13;

    private final double confirmationTime = 0.2;

    private final double fastSpeed = 5; //ft/s
    private final double slowSpeed = 2; //ft/s
    private final double adjustmentSpeed = 0.5;

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

    public AutoBalance() {
        state = MoveState.TOWARDS_CHARGE_STATION;
        tickCounter = 0;
    }

    public static double getTilt() {
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

        double speed = 0;

        if (state == MoveState.TOWARDS_CHARGE_STATION) {
            if (getTilt() > chargeStationDegree) {
                tickCounter++;
            }

            speed = fastSpeed;

            if (tickCounter > secondsToTicks(confirmationTime)) {
                state = MoveState.SLOW_UP;
                tickCounter = 0;

                speed = slowSpeed;
            }
        } else if (state == MoveState.SLOW_UP) {
            if (getTilt() < levelDegree) {
                tickCounter++;
            }

            if (tickCounter > secondsToTicks(confirmationTime)) {
                state = MoveState.BALANCE;
                tickCounter = 0;

                speed = 0;
            }
        } else if (state == MoveState.BALANCE) {
            if (Math.abs(getTilt()) <= levelDegree / 2) {
                tickCounter++;
            }

            if (getTilt() >= levelDegree) {
                speed = -adjustmentSpeed;
            } else if (getTilt() <= -levelDegree) {
                speed = adjustmentSpeed;
            }

            if (tickCounter > secondsToTicks(confirmationTime)) {
                state = MoveState.IDLE;
                tickCounter = 0;
                speed = 0;
            }
        } else {
            speed = 0;
        }

        if (Math.abs(speed) > 0.01) {
            drive.getSwerveDrive().drive(
                new Translation2d(
                    speed, 
                    0
                ), 
                0, 
                true, 
                Robot.isReal()
            );
        } else {
            drive.getSwerveDrive().stop();
        }

        SmartDashboard.putString("AutoBalance", state.name());
        SmartDashboard.updateValues();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
