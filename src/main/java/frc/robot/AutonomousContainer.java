package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.IntakeCommand;
import frc.robot.commands.auto.PlacePiece;
import frc.robot.commands.auto.AutoBalance.MoveState;
import frc.robot.commands.auto.arm.ArmSlightDownwardsPlace;
import frc.robot.commands.auto.arm.MoveArmToPosition;
import frc.robot.commands.auto.arm.StopArmMovement;
import frc.robot.commands.auto.general.Wait;
import frc.robot.commands.auto.movement.PIDRotate;
import frc.robot.commands.auto.movement.RotateTo;
import frc.robot.commands.auto.movement.SimpleForwards;
import frc.robot.commands.auto.movement.SmallRotate;
import frc.robot.commands.auto.movement.StopDrive;
import frc.robot.commands.auto.movement.StupidRotate;
import frc.robot.commands.auto.placing.AutoShoot;
import frc.robot.commands.auto.placing.AutoShoot.ShootType;
import frc.robot.subsystems.arm.ArmSubsystem.ArmState;
import frc.robot.subsystems.intake.GrabberSubsystem;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.pathing.PathConjugate;
import frc.team_8840_lib.pathing.PathPlanner;

public class AutonomousContainer {
    public static class Files {
        public static final String FOLDER = "8840appdata";
        public static class General {
            public static class Testing {
                public static final String SEG_1 = "testing_seg_1.json";
                public static final String SEG_2 = "testing_seg_2.json";
            }
            public static class TopCorner {
                public static final String SEG_1 = "top_seg_1.json";
                public static final String SEG_2 = "top_seg_2.json";
                public static final String SEG_3 = "top_seg_3.json";
            }
            public static class Basic {
                public static final String Bottom_Corner = "easy_exit_bottom_REAL.json";
                public static final String Red_Bottom_Corner = "easy_exit_bottom_red.json";
                public static final String Top_Corner = "easy_exit_top_REAL.json";
            }
            public static class ChargeStation {
                public static final String Engage = "on_to_charge.json";
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

    private static PathConjugate[] place(ArmState location) {
        return new PathConjugate[] {
            PathConjugate.command(new Wait(300)),
            PathConjugate.command(new MoveArmToPosition(location)),
            PathConjugate.command(new ArmSlightDownwardsPlace()),
            PathConjugate.command(new StopArmMovement()),
            PathConjugate.command(new Wait(100)),
            PathConjugate.command(new PlacePiece()),
            PathConjugate.runOnce(() -> {
                GrabberSubsystem.getInstance().stopOuttake();
            }),
            PathConjugate.command(new MoveArmToPosition(ArmState.RESTING))
        };
    }

    private static PathConjugate[] intake() {
        return new PathConjugate[] {
            PathConjugate.command(new MoveArmToPosition(ArmState.INTAKE_FLOOR)),
            PathConjugate.command(new ArmSlightDownwardsPlace()),
            PathConjugate.command(new StopArmMovement()),
            PathConjugate.command(new Wait(100)),
            PathConjugate.command(new IntakeCommand(500)),
            PathConjugate.runOnce(() -> {
                GrabberSubsystem.getInstance().stopOuttake();
            }),
            PathConjugate.command(new MoveArmToPosition(ArmState.RESTING))
        };
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

        PathPlanner.addAuto("Place_High", new PathPlanner(
            place(ArmState.PLACING_UPPER_CONE)
        ));

        PathPlanner.addAuto("Place_Mid", new PathPlanner(
            place(ArmState.PLACING_MIDDLE_CONE)
        ));

        PathPlanner.addAuto("FullTopCorner", new PathPlanner(
            //Move the arm up
            place(ArmState.PLACING_UPPER_CONE),
            //Reset the drive base so it'll actually move
            new PathConjugate[] {
                PathConjugate.runOnce(() -> {
                    RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
                }),
                PathConjugate.command(new SmallRotate()),
                PathConjugate.command(new Wait(400)),
                PathConjugate.loadPathFromFile(Path.of(home, Files.FOLDER, Files.General.TopCorner.SEG_1)),
                PathConjugate.command(new StopDrive()),
                PathConjugate.command(new RotateTo(Rotation2d.fromDegrees(170), Rotation2d.fromDegrees(5))),
                PathConjugate.command(new StopDrive()),
            },
            intake(),
            new PathConjugate[] {
                //PathConjugate.command(new StupidRotate(Rotation2d.fromDegrees(5))),
                //PathConjugate.command(new RotateTo(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(5))),
                //PathConjugate.command(new StopDrive()),
                PathConjugate.command(new StupidRotate(Rotation2d.fromDegrees(5))),
                PathConjugate.command(new StopDrive()),
                PathConjugate.loadPathFromFile(Path.of(home, Files.FOLDER, Files.General.TopCorner.SEG_2)),
                PathConjugate.command(new StopDrive()),
            },
            place(ArmState.PLACING_MIDDLE_CONE)
            // new PathConjugate[] {
            //     PathConjugate.loadPathFromFile(Path.of(home, Files.FOLDER, Files.General.TopCorner.SEG_3)),
            //     PathConjugate.command(new StopDrive()),
            //     onFinish
            // }
        ));

        PathPlanner.addAuto("Testing", new PathPlanner(
            PathConjugate.runOnce(() -> {
                RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
            }),
            PathConjugate.waitForPath(),
            PathConjugate.command(new StopDrive()),
            onFinish
        ));

        PathPlanner.addAuto("Rotation", new PathPlanner(
            PathConjugate.runOnce(() -> {
                RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
            }),
            PathConjugate.command(new SmallRotate()),
            PathConjugate.command(new Wait(400)),
            PathConjugate.command(new StupidRotate(Rotation2d.fromDegrees(5))),
            PathConjugate.command(new StopDrive()),
            onFinish
        ));

        PathPlanner.addAuto("Upper_Cone_Top", new PathPlanner(
            place(ArmState.PLACING_UPPER_CONE),
            //Reset the drive base so it'll actually move
            new PathConjugate[] {
                PathConjugate.runOnce(() -> {
                    RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
                }),
                PathConjugate.command(new SmallRotate()),
                PathConjugate.command(new Wait(400)),
                PathConjugate.loadPathFromFile(Path.of(home, Files.FOLDER, Files.General.Basic.Top_Corner)),
                PathConjugate.command(new StopDrive()),
                onFinish
            }
        ));

        PathPlanner.addAuto("Middle_Cone_Top", new PathPlanner(
            place(ArmState.PLACING_MIDDLE_CONE),
            //Reset the drive base so it'll actually move
            new PathConjugate[] {
                PathConjugate.runOnce(() -> {
                    RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
                }),
                PathConjugate.command(new SmallRotate()),
                PathConjugate.command(new Wait(400)),
                PathConjugate.loadPathFromFile(Path.of(home, Files.FOLDER, Files.General.Basic.Top_Corner)),
                PathConjugate.command(new StopDrive()),
                onFinish
            }
        ));

        PathPlanner.addAuto("Top_Cone_Bottom", new PathPlanner(
            place(ArmState.PLACING_UPPER_CONE),
            //Reset the drive base so it'll actually move
            new PathConjugate[] {
                PathConjugate.runOnce(() -> {
                    RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
                }),
                PathConjugate.command(new SmallRotate()),
                PathConjugate.command(new Wait(400)),
                PathConjugate.loadPathFromFile(Path.of(home, Files.FOLDER, Files.General.Basic.Bottom_Corner)),
                PathConjugate.command(new StopDrive()),
                PathConjugate.command(new SimpleForwards(1500)),
                PathConjugate.command(new StopDrive()),
                onFinish
            }
        ));

        PathPlanner.addAuto("RED_Top_Cone_Bottom", new PathPlanner(
            place(ArmState.PLACING_UPPER_CONE),
            //Reset the drive base so it'll actually move
            new PathConjugate[] {
                PathConjugate.runOnce(() -> {
                    RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
                }),
                PathConjugate.command(new SmallRotate()),
                PathConjugate.command(new Wait(400)),
                PathConjugate.loadPathFromFile(Path.of(home, Files.FOLDER, Files.General.Basic.Red_Bottom_Corner)),
                PathConjugate.command(new StopDrive()),
                onFinish
            }
        ));

        PathPlanner.addAuto("Bottom_Cone_Bottom", new PathPlanner(
            place(ArmState.PLACING_MIDDLE_CONE),
            //Reset the drive base so it'll actually move
            new PathConjugate[] {
                PathConjugate.runOnce(() -> {
                    RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
                }),
                PathConjugate.command(new SmallRotate()),
                PathConjugate.command(new Wait(400)),
                PathConjugate.loadPathFromFile(Path.of(home, Files.FOLDER, Files.General.Basic.Bottom_Corner)),
                PathConjugate.command(new StopDrive()),
                PathConjugate.command(new SimpleForwards(1500)),
                PathConjugate.command(new StopDrive()),
                onFinish
            }
        ));

        PathPlanner.addAuto("Bottom_Just_Movement", new PathPlanner(
            PathConjugate.runOnce(() -> {
                RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
            }),
            PathConjugate.command(new SmallRotate()),
            PathConjugate.command(new Wait(400)),
            PathConjugate.loadPathFromFile(Path.of(home, Files.FOLDER, Files.General.Basic.Bottom_Corner)),
            PathConjugate.command(new StopDrive()),
            PathConjugate.command(new SimpleForwards(1500)),
            PathConjugate.command(new StopDrive()),
            onFinish
        ));

        PathPlanner.addAuto("Top_Just_Movement", new PathPlanner(
            PathConjugate.runOnce(() -> {
                RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
            }),
            PathConjugate.command(new SmallRotate()),
            PathConjugate.command(new Wait(400)),
            PathConjugate.loadPathFromFile(Path.of(home, Files.FOLDER, Files.General.Basic.Top_Corner)),
            PathConjugate.command(new StopDrive()),
            onFinish
        ));

        PathPlanner.addAuto("Just_Engage", new PathPlanner(
            PathConjugate.runOnce(() -> {
                RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
            }),
            PathConjugate.command(new SmallRotate()),
            PathConjugate.command(new Wait(400)),
            PathConjugate.loadPathFromFile(Path.of(home, Files.FOLDER, Files.General.ChargeStation.Engage)),
            PathConjugate.command(new StopDrive()),
            onFinish
        ));

        PathPlanner.addAuto("Top_Cone_Then_Engage", new PathPlanner(
            place(ArmState.PLACING_UPPER_CONE),
            new PathConjugate[] {
                PathConjugate.runOnce(() -> {
                    RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
                }),
                PathConjugate.command(new SmallRotate()),
                PathConjugate.command(new Wait(400)),
                PathConjugate.loadPathFromFile(Path.of(home, Files.FOLDER, Files.General.ChargeStation.Engage)),
                PathConjugate.command(new StopDrive()),
                onFinish
            }
        ));

        PathPlanner.addAuto("Just_Balance", new PathPlanner(
            PathConjugate.runOnce(() -> {
                RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
            }),
            PathConjugate.command(new SmallRotate()),
            PathConjugate.command(new Wait(400)),
            PathConjugate.command(new AutoBalance()),
            onFinish
        ));

        PathPlanner.addAuto("Top_Cone_Then_Balance", new PathPlanner(
            place(ArmState.PLACING_UPPER_CONE),
            new PathConjugate[] {
                PathConjugate.runOnce(() -> {
                    RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
                }),
                PathConjugate.command(new SmallRotate()),
                PathConjugate.command(new Wait(400)),
                PathConjugate.command(new AutoBalance()),
                onFinish
            }
        ));

        //TODO: REMOVE
        PathPlanner.selectAuto("Place_High");
    }

    static void updateAboutAutonomousLocation() {
        SmartDashboard.putString("Auto Location", PathPlanner.getSelectedAutoName());
        SmartDashboard.putString("Autonomous Status", "Started!");
    }
}
