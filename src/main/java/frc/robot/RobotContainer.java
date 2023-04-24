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

        //Create new instances of all of the subsystems.
        driveSubsystem = new DriveSubsystem();
        visionSubsystem = new VisionSubsystem();
        armSubsystem = new ArmSubsystem();
        grabberSubsystem = new GrabberSubsystem();

        //Set the default command for the drive subsystem to XboxDrive.
        driveSubsystem.setDefaultCommand(
            new XboxDrive(driveSubsystem)
        );

        //Create a new PS4Operator command since we're using it in multiple places.
        PS4Operator operator = new PS4Operator(grabberSubsystem, armSubsystem);

        //Set the default commands for grabber and arm to the PS4Operator command.
        grabberSubsystem.setDefaultCommand(
            operator
        );

        armSubsystem.setDefaultCommand(
            operator
        );
    }

    public DriveSubsystem getDriveSubsystem() {
        return driveSubsystem;
    }

    public void onPhaseChange() {
        //On game phase, change the brake mode of the drive subsystem.
        //Also, reset the odometry if we're in autonomous.
        GamePhase phase = GamePhase.getCurrentPhase();

        if (phase == GamePhase.Autonomous) {
            driveSubsystem.setBrakeMode(BrakeMode.NORMAL);
            driveSubsystem.resetOdometry();
        } else if (phase == GamePhase.Teleop) {
            driveSubsystem.setBrakeMode(BrakeMode.NORMAL);
            driveSubsystem.getSwerveDrive().triggerNoOptimization();
        }
    }
}
