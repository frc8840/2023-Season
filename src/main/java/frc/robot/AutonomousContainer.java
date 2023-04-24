package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.IntakeCommand;
import frc.robot.commands.auto.PlacePiece;
import frc.robot.commands.auto.arm.ArmSlightDownwardsPlace;
import frc.robot.commands.auto.arm.MoveArmToPosition;
import frc.robot.commands.auto.arm.StopArmMovement;
import frc.robot.commands.auto.general.Wait;
import frc.robot.commands.auto.movement.RotateTo;
import frc.robot.commands.auto.movement.SimpleForwards;
import frc.robot.commands.auto.movement.SmallRotate;
import frc.robot.commands.auto.movement.StopDrive;
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

        /**
         * This auto shoots mid, then drives forward.
         * This is an old auto that we don't use anymore since we changed to cone only.
         */
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

        /**
         * This auto moves the arm to the high position.
         * This auto was generally used in the middle position, or the bottom.
         */
        PathPlanner.addAuto("Place_High", new PathPlanner(
            place(ArmState.PLACING_UPPER_CONE)
        ));

        /**
         * This auto moves the arm to the middle position.
         */
        PathPlanner.addAuto("Place_Mid", new PathPlanner(
            place(ArmState.PLACING_MIDDLE_CONE)
        ));

        /**
         * This auto is generally testing and was not used in the season.
         * This auto is (in theory) a two piece auto, but we never got the time to test it.
         * It was not used in competition.
         */
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
                PathConjugate.command(new RotateTo(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(5))),
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

        /**
         * This auto is meant for testing. This allowed the user to load paths, then test them without having to change code.
         */
        PathPlanner.addAuto("Testing", new PathPlanner(
            PathConjugate.runOnce(() -> {
                RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
            }),
            PathConjugate.waitForPath(),
            PathConjugate.command(new StopDrive()),
            onFinish
        ));

        /**
         * Rotation testing.
         */
        PathPlanner.addAuto("Rotation", new PathPlanner(
            PathConjugate.runOnce(() -> {
                RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
            }),
            PathConjugate.command(new SmallRotate()),
            PathConjugate.command(new Wait(400)),
            PathConjugate.command(new RotateTo(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(5))),
            PathConjugate.command(new StopDrive()),
            onFinish
        ));
        
        /**
         * Place the code on the top row, then taxi out (substation side).
         */
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

        /**
         * Place the code on the middle row, then taxi out (substation side).
         */
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

        /**
         * Place the code on the top row, then taxi out (bottom side).
         */
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

        /**
         * Due to issues with the planner code, a seperate path had to be made for the red side.
         * As of 4/23/23, the fix has not been released yet, but will be soon.
         */
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

        /**
         * Place the code on the middle row, then taxi out (bottom side).
         * It was accidentally misnamed. It's actually the middle row.
         */
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

        /**
         * Bottom side, just movement.
         * Never used.
         */
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

        /**
         * Top side, just movement.
         */
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

        /**
         * A path that should just engage the charge station.
         */
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

        /**
         * Place the code on the top row, then engage the charge station.
         */
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

        /**
         * Place the code on the middle row, then engage the charge station.
         */
        PathPlanner.addAuto("Just_Balance", new PathPlanner(
            PathConjugate.runOnce(() -> {
                RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().triggerNoOptimization();
            }),
            PathConjugate.command(new SmallRotate()),
            PathConjugate.command(new Wait(400)),
            PathConjugate.command(new AutoBalance()),
            onFinish
        ));

        /**
         * Place the code on the middle row, then balance on the charge station.
         */
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

        //This sets the default autonomous/selects it.
        //This is useful if (Jaiden) is forgetful.
        PathPlanner.selectAuto("Place_High");
    }

    /**
     * This method just updates the SmartDashboard with the current autonomous.
     * It's called in a few places.
     */
    static void updateAboutAutonomousLocation() {
        SmartDashboard.putString("Auto Location", PathPlanner.getSelectedAutoName());
        SmartDashboard.putString("Autonomous Status", "Started!");
    }
}
