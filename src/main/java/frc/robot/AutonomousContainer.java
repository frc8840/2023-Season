package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.IntakeCommand;
import frc.robot.commands.auto.PlacePiece;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.pathing.PathConjugate;
import frc.team_8840_lib.pathing.PathPlanner;

public class AutonomousContainer {
    public static class Files {
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

        PathPlanner.addAuto(PathPlanner.AUTOS.ChargedUp.BLUE_LOWER, new PathPlanner(
            PathConjugate.runOnce(() -> {
                updateAboutAutonomousLocation();
                Logger.Log("[Auto] Started Blue Lower Auto!");
            }),
            PathConjugate.loadPathFromFile(Path.of(
                home, "8840appdata", 
                AutonomousContainer.Files.Blue.Lower.SEG_1
            )),
            PathConjugate.command(new IntakeCommand()),
            PathConjugate.loadPathFromFile(Path.of(
                home, "8840appdata", 
                AutonomousContainer.Files.Blue.Lower.SEG_2
            )),
            PathConjugate.command(new PlacePiece()),
            onFinish
        ));

        PathPlanner.addAuto(PathPlanner.AUTOS.ChargedUp.BLUE_MIDDLE, new PathPlanner(
            PathConjugate.runOnce(() -> {
                updateAboutAutonomousLocation();
                Logger.Log("[Auto] Started Blue Middle Auto!");
            }),
            PathConjugate.loadPathFromFile(Path.of(
                home, "8840appdata", 
                AutonomousContainer.Files.Blue.Middle.SEG_1
            )),
            PathConjugate.loadPathFromFile(Path.of(
                home, "8840appdata", 
                AutonomousContainer.Files.Blue.Middle.SEG_2
            )),
            onFinish
        ));

        PathPlanner.addAuto(PathPlanner.AUTOS.ChargedUp.BLUE_UPPER, new PathPlanner(
            PathConjugate.runOnce(() -> {
                updateAboutAutonomousLocation();
                Logger.Log("[Auto] Started Blue Upper Auto!");
            }),
            onFinish
        ));

        PathPlanner.addAuto(PathPlanner.AUTOS.ChargedUp.RED_LOWER, new PathPlanner(
            PathConjugate.runOnce(() -> {
                updateAboutAutonomousLocation();
                Logger.Log("[Auto] Started Red Lower Auto!");
            }),
            onFinish
        ));

        PathPlanner.addAuto(PathPlanner.AUTOS.ChargedUp.RED_MIDDLE, new PathPlanner(
            PathConjugate.runOnce(() -> {
                updateAboutAutonomousLocation();
                Logger.Log("[Auto] Started Red Middle Auto!");
            }),
            onFinish
        ));

        PathPlanner.addAuto(PathPlanner.AUTOS.ChargedUp.RED_UPPER, new PathPlanner(
            PathConjugate.runOnce(() -> {
                updateAboutAutonomousLocation();
                Logger.Log("[Auto] Started Red Upper Auto!");
            }),
            onFinish
        ));

        //TODO: REMOVE
        PathPlanner.selectAuto(PathPlanner.AUTOS.ChargedUp.BLUE_LOWER);
    }

    static void updateAboutAutonomousLocation() {
        SmartDashboard.putString("Auto Location", PathPlanner.getSelectedAutoName());
        SmartDashboard.putString("Autonomous Status", "Started!");
    }
}
