package frc.robot;

import frc.robot.commands.PS4Drive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.BrakeMode;
import frc.team_8840_lib.utils.GamePhase;

public class RobotContainer {
    private DriveSubsystem driveSubsystem;

    public RobotContainer() {
        driveSubsystem = new DriveSubsystem();

        driveSubsystem.setDefaultCommand(
            new PS4Drive(driveSubsystem)
        );
    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    public void onPhaseChange() {
        GamePhase phase = GamePhase.getCurrentPhase();

        if (phase == GamePhase.Autonomous) {
            driveSubsystem.setBrakeMode(BrakeMode.NORMAL);
            driveSubsystem.resetOdometry();
        } else if (phase == GamePhase.Teleop) {
            driveSubsystem.setBrakeMode(BrakeMode.NORMAL);
        }
    }
}
