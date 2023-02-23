package frc.robot;

import frc.robot.commands.EstimatePosition;
import frc.robot.commands.OperateArm;
import frc.robot.commands.PS4Drive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.BrakeMode;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.team_8840_lib.utils.GamePhase;

public class RobotContainer {
    private static RobotContainer instance;
    public static RobotContainer getInstance() {
        return instance;
    }

    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;
    private ArmSubsystem armSubsystem;

    public RobotContainer() {
        instance = this;

        driveSubsystem = new DriveSubsystem();
        visionSubsystem = new VisionSubsystem();
        //armSubsystem = new ArmSubsystem();

        driveSubsystem.setDefaultCommand(
            new PS4Drive(driveSubsystem)
        );

        visionSubsystem.setDefaultCommand(
            new EstimatePosition(visionSubsystem)
        );

        // armSubsystem.setDefaultCommand(
        //     new OperateArm(armSubsystem)
        // );
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
