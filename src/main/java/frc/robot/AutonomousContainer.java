package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.IntakeCommand;
import frc.robot.commands.auto.PlacePiece;
import frc.robot.commands.auto.general.Wait;
import frc.robot.commands.auto.movement.SimpleForwards;
import frc.robot.commands.auto.movement.SmallRotate;
import frc.robot.commands.auto.movement.StopDrive;
import frc.robot.commands.auto.placing.AutoShoot;
import frc.robot.commands.auto.placing.AutoShoot.ShootType;
import frc.robot.subsystems.intake.GrabberSubsystem;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.pathing.PathConjugate;
import frc.team_8840_lib.pathing.PathPlanner;

public class AutonomousContainer {
    public static class Files {
        public static class General {
            public static class Testing {
                public static final String SEG_1 = "testing_seg_1.json";
                public static final String SEG_2 = "testing_seg_2.json";
            }
            public static class TopCorner {
                public static final String SEG_1 = "top_corner_seg_1.json";
                public static final String SEG_2 = "top_corner_seg_2.json";
            }
        }

        public static class Blue {
            public static class Lower {
                public static final String SEG_1 = "auton/blue_lower/seg_1.json";
                public static final String SEG_2 = "auton/blue_lower/seg_2.json";
            }
            public static class Middle {
                public static final String SEG_1 = "auton/blue_mid/seg_1.json";
                public static final String SEG_2 = "auton/blue_mid/seg_2.json";
            }
            public static class Upper {
                public static final String SEG_1 = "auton/blue_upper/seg_1.json";
                public static final String SEG_2 = "auton/blue_upper/seg_2.json";
            }
        }
    }

    public static void init() throws IOException {
        PathConjugate onFinish = PathConjugate.runOnce(() -> {
            Logger.Log("[Auto] Finished Auto!");
            SmartDashboard.putString("Autonomous Status", "Finished");
        });
        
        String home = System.getProperty("user.home");

        PathPlanner.addAuto("Mid_Shoot_Forward", new PathPlanner(
            PathConjugate.runOnce(() -> {
                SmartDashboard.putString("Auto", "Started");
            }),
            PathConjugate.command(new AutoShoot(ShootType.MID)),
            PathConjugate.runOnce(() -> {
                GrabberSubsystem.getInstance().stopOuttake();
            }),
            PathConjugate.command(new SimpleForwards(2000)),
            PathConjugate.command(new StopDrive()),
            onFinish
        ));

        PathPlanner.addAuto("TopCorner", new PathPlanner(
            //Move the arm up
            PathConjugate.runOnce(() -> {
                Logger.Log("[Auto] TODO: Move arm up!");
            }),
            PathConjugate.command(new AutoShoot(ShootType.MID)),
            PathConjugate.runOnce(() -> {
                GrabberSubsystem.getInstance().stopOuttake();
            }),
            PathConjugate.runOnce(() -> {
                Logger.Log("[Auto] TODO: Move arm down!");
            }),
            //Reset the drive base so it'll actually move
            PathConjugate.runOnce(() -> {
                RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
            }),
            PathConjugate.command(new SmallRotate()),
            PathConjugate.command(new Wait(400)),
            PathConjugate.loadPathFromFile(Path.of(home, Files.General.Testing.SEG_1)),
            PathConjugate.runOnce(() -> {
                Logger.Log("TODO: Arm Intake!");
            }),
            PathConjugate.command(new IntakeCommand(200)),
            PathConjugate.runOnce(() -> {
                GrabberSubsystem.getInstance().stopOuttake();
            }),
            PathConjugate.runOnce(() -> {
                Logger.Log("TODO: Arm back!");
            }),
            PathConjugate.loadPathFromFile(Path.of(home, Files.General.Testing.SEG_2)),
            PathConjugate.runOnce(() -> {
                Logger.Log("TODO: Arm place!");
            }),
            PathConjugate.command(new AutoShoot(ShootType.MID)),
            PathConjugate.runOnce(() -> {
                GrabberSubsystem.getInstance().stopOuttake();
            }),
            onFinish
        ));

        PathPlanner.addAuto("Testing", new PathPlanner(
            PathConjugate.runOnce(() -> {
                RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
            }),
            PathConjugate.waitForPath(),
            PathConjugate.command(new StopDrive()),
            onFinish
        ));

        //TODO: REMOVE
        PathPlanner.selectAuto("TopCorner");
    }

    static void updateAboutAutonomousLocation() {
        SmartDashboard.putString("Auto Location", PathPlanner.getSelectedAutoName());
        SmartDashboard.putString("Autonomous Status", "Started!");
    }
}
