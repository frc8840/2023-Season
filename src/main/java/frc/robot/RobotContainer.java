package frc.robot;

import frc.robot.commands.EstimatePosition;
import frc.robot.commands.OperateArm;
import frc.robot.commands.PS4Operator;
import frc.robot.commands.XboxDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.BrakeMode;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.GrabberSubsystem;
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
    private GrabberSubsystem grabberSubsystem;

    public RobotContainer() {
        instance = this;

        driveSubsystem = new DriveSubsystem();
        //visionSubsystem = new VisionSubsystem();
        armSubsystem = new ArmSubsystem();
        grabberSubsystem = new GrabberSubsystem();

        driveSubsystem.setDefaultCommand(
            new XboxDrive(driveSubsystem)
        );

        // visionSubsystem.setDefaultCommand(
        //     new EstimatePosition(visionSubsystem)
        // );

        PS4Operator operator = new PS4Operator(grabberSubsystem, armSubsystem);

        // grabberSubsystem.setDefaultCommand(
        //     operator
        // );

        armSubsystem.setDefaultCommand(
            operator
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
